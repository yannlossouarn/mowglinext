#!/usr/bin/env python3
"""Path-following calibration assistant (interactive TUI).

Evaluates how accurately the robot tracks a REALISTIC planned path with the
local planner (FTC / FollowPath), so you can tune the drive feedforward
(fc/fv/fb) and the controller anti-hunt knobs (kd_lat, derivative LPF, gyro
rate loop) against real cross-track / heading / hunting numbers.

It drives two outlines around the robot's current pose -- one clockwise, one
counter-clockwise -- each as a sequence of real Smac (Hybrid-A*) legs handed to
FollowPath (NOT hand-built straight lines, which give artifact hunting). For
each run it reports, against the actual planned polyline:
    * cross-track RMS / peak      (tracking accuracy)
    * heading-error RMS / peak    (yaw tracking)
    * cmd wz zero-cross rate      (steering hunting)
    * gyro_z zero-cross rate      (measured oscillation)
    * mean speed                  (fv / speed-tracking)
then compares CW vs CCW (directional asymmetry) and proposes the next knob to
turn.

Knobs it can set live:
    fc/fv/fb            -> /hardware_bridge ff_*_byte           (NaN = default)
    kd_lat, deriv LPF   -> /controller_server FollowPath.{kd_lat,derivative_filter_tau}
    gyro rate loop      -> /hardware_bridge angular_rate_loop_enabled

PREREQUISITES (header flags them):
  * BT must be in a driving mode so Nav2 cmd_vel reaches the motors WITHOUT the
    blade -> this tool can put it in RECORDING (the proven test-drive mode).
  * Localization must be converged (sigma_xy small) and yaw anchored (drive a
    straight leg first if you just restarted the localizer).
  * Robot OFF the dock, blade OFF, area clear -- it drives autonomously.

Run from the host:
    ./docker/path_calibrate.sh
  or:
    docker exec -it mowgli-ros2 bash -lc \\
      'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \\
       exec python3 /ros2_ws/maps/path_calibrate.py'
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav2_msgs.srv import ClearEntireCostmap
from builtin_interfaces.msg import Duration
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Status
from mowgli_interfaces.srv import GetMowingArea, HighLevelControl

HL_NAMES = {0: "NULL", 1: "IDLE", 2: "AUTONOMOUS", 3: "RECORDING", 4: "MANUAL_MOWING"}
HL_DRIVES = {2, 3, 4}  # firmware accepts cmd_vel out of IDLE/NULL
HB = "/hardware_bridge"
CS = "/controller_server"


class Col:
    R = "\033[0m"
    B = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[31m"
    GRN = "\033[32m"
    YEL = "\033[33m"
    BLU = "\033[34m"
    MAG = "\033[35m"
    CYN = "\033[36m"


def c(s, col):
    return f"{col}{s}{Col.R}"


def clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def yq(y):
    return math.sin(y / 2.0), math.cos(y / 2.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pt_seg(px, py, a, b):
    """Distance from (px,py) to segment a->b, and the segment tangent yaw."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    tan = math.atan2(dy, dx)
    if L2 == 0.0:
        return math.hypot(px - a[0], py - a[1]), tan
    t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy)), tan


def signed_area(v):
    s = 0.0
    n = len(v)
    for i in range(n):
        x1, y1 = v[i]
        x2, y2 = v[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def inset_poly(v, margin):
    """Shrink a polygon toward its centroid by ~margin metres (approx)."""
    if margin <= 0.0 or len(v) < 3:
        return v
    cx = sum(p[0] for p in v) / len(v)
    cy = sum(p[1] for p in v) / len(v)
    ar = sum(math.hypot(p[0] - cx, p[1] - cy) for p in v) / len(v)
    if ar <= margin:
        return v
    sc = (ar - margin) / ar
    return [(cx + (p[0] - cx) * sc, cy + (p[1] - cy) * sc) for p in v]


def order_loop(verts, robot_xy, direction, margin=0.0):
    """Ordered closed vertex list starting at the vertex nearest the robot,
    walking CW or CCW (uses the polygon's signed area to pick winding)."""
    v = inset_poly(verts, margin)
    n = len(v)
    i0 = min(range(n), key=lambda i: math.hypot(v[i][0] - robot_xy[0], v[i][1] - robot_xy[1]))
    ccw_stored = signed_area(v) > 0
    if direction == "ccw":
        step = 1 if ccw_stored else -1
    else:
        step = -1 if ccw_stored else 1
    seq = [v[(i0 + step * k) % n] for k in range(n)]
    seq.append(seq[0])  # close the loop (last leg returns to start vertex)
    return seq


def zc(vals, ts, mask):
    """Zero-crossings of vals over masked (moving) samples + rate per second."""
    n = 0
    t0 = t1 = None
    for i in range(1, len(vals)):
        if not (mask[i - 1] and mask[i]):
            continue
        t0 = ts[i - 1] if t0 is None else t0
        t1 = ts[i]
        if vals[i - 1] * vals[i] < 0:
            n += 1
    dur = (t1 - t0) if (t0 and t1) else 0.0
    return (n / dur if dur else 0.0), n, dur


def seg_turn_flags(poly, win_m=0.5, ang_thr=0.35):
    """Per-segment True if that part of the path is a TURN (cumulative heading
    change > ang_thr over ~win_m of arc on each side). Geometry-based, so it is
    independent of how the robot actually responded."""
    n = len(poly)
    if n < 3:
        return [False] * max(0, n - 1)
    sd, sl = [], []
    for i in range(n - 1):
        dx, dy = poly[i + 1][0] - poly[i][0], poly[i + 1][1] - poly[i][1]
        sd.append(math.atan2(dy, dx))
        sl.append(math.hypot(dx, dy))
    flags = [False] * (n - 1)
    for i in range(n - 1):
        acc, d, j = 0.0, 0.0, i
        while j < n - 2 and d < win_m:
            acc += abs(wrap(sd[j + 1] - sd[j]))
            d += sl[j + 1]
            j += 1
        d, j = 0.0, i
        while j > 0 and d < win_m:
            acc += abs(wrap(sd[j] - sd[j - 1]))
            d += sl[j]
            j -= 1
        flags[i] = acc > ang_thr
    return flags


def _rms(a):
    return math.sqrt(sum(x * x for x in a) / len(a)) if a else 0.0


def analyze(samples, poly):
    """Cross-track / heading / hunting metrics vs the planned polyline, split
    into STRAIGHT and TURN regimes (path-curvature classified)."""
    if len(samples) < 5 or len(poly) < 2:
        return None
    flags = seg_turn_flags(poly)
    ts = [s[0] for s in samples]
    wz = [s[5] for s in samples]
    gz = [s[6] for s in samples]
    moving = [abs(s[4]) > 0.08 for s in samples]
    straight_mask = [False] * len(samples)
    ct_all, herr_all, spd = [], [], []
    s_ct, s_herr, t_ct, t_herr, t_wz = [], [], [], [], []
    for k, (s, mv) in enumerate(zip(samples, moving)):
        if not mv:
            continue
        best_d, best_tan, best_i = 1e9, 0.0, 0
        for i in range(len(poly) - 1):
            d, tan = pt_seg(s[1], s[2], poly[i], poly[i + 1])
            if d < best_d:
                best_d, best_tan, best_i = d, tan, i
        he = abs(math.degrees(wrap(s[3] - best_tan)))
        ct_all.append(best_d)
        herr_all.append(he)
        spd.append(abs(s[7]))  # wheel-odom ACHIEVED forward speed
        if flags[best_i]:
            t_ct.append(best_d)
            t_herr.append(he)
            t_wz.append(abs(s[5]))
        else:
            s_ct.append(best_d)
            s_herr.append(he)
            straight_mask[k] = True
    if not ct_all:
        return None
    wz_rate, _, dur = zc(wz, ts, moving)
    gz_rate, _, _ = zc(gz, ts, moving)
    s_wz_rate, _, _ = zc(wz, ts, straight_mask)  # hunting on STRAIGHTS only
    s_gz_rate, _, _ = zc(gz, ts, straight_mask)
    ss = sorted(spd)
    cruise = ss[min(len(ss) - 1, int(0.9 * len(ss)))]
    return {
        # overall
        "ct_rms": _rms(ct_all), "ct_peak": max(ct_all),
        "herr_rms": _rms(herr_all), "herr_peak": max(herr_all),
        "wz_zc": wz_rate, "gz_zc": gz_rate,
        "wz_peak": max((abs(w) for w, mv in zip(wz, moving) if mv), default=0.0),
        "speed": cruise, "speed_mean": sum(spd) / len(spd), "dur": dur, "n": len(ct_all),
        # STRAIGHT regime (tracking + hunting live here)
        "s_ct_rms": _rms(s_ct), "s_ct_peak": max(s_ct, default=0.0),
        "s_herr_rms": _rms(s_herr), "s_wz_zc": s_wz_rate, "s_gz_zc": s_gz_rate,
        "s_n": len(s_ct),
        # TURN regime (cornering overshoot lives here)
        "t_ct_peak": max(t_ct, default=0.0), "t_herr_peak": max(t_herr, default=0.0),
        "t_wz_peak": max(t_wz, default=0.0), "t_n": len(t_ct),
    }


def analyze_spin(gyro_buf, target_rad, baseline=0.0):
    """In-place rotation metrics from HIGH-RATE IMU gyro integration (the 10 Hz
    fused pose can't resolve a sub-second small pivot). `gyro_buf` is a list of
    (t, gyro_z); `baseline` is the stationary gyro bias to subtract. Returns
    achieved vs intended yaw, overshoot, undershoot, settle, oscillation."""
    if len(gyro_buf) < 6:
        return None
    ts = [g[0] for g in gyro_buf]
    cum = [0.0]
    revs = 0
    prev = None
    for i in range(1, len(gyro_buf)):
        dt = ts[i] - ts[i - 1]
        rate = gyro_buf[i][1] - baseline
        if dt <= 0 or dt > 0.3:
            cum.append(cum[-1])
            continue
        cum.append(cum[-1] + rate * dt)  # integrate bias-corrected yaw rate
        if prev is not None and prev * rate < 0 and abs(rate) > 0.05:
            revs += 1
        prev = rate
    achieved = cum[-1]                                      # FINAL resting yaw
    sgn = 1.0 if target_rad >= 0 else -1.0
    peak = max(cum) if sgn > 0 else min(cum)                # furthest rotated
    overshoot = max(0.0, (peak - target_rad) * sgn)         # peak went past target
    undershoot = max(0.0, (target_rad - achieved) * sgn)    # rest stopped short
    backcreep = max(0.0, (peak - achieved) * sgn)           # crept back from peak after stop
    err = target_rad - achieved
    tol = math.radians(0.5)
    settle = ts[-1] - ts[0]
    for i in range(len(cum)):
        if all(abs(cum[k] - target_rad) <= tol for k in range(i, len(cum))):
            settle = ts[i] - ts[0]
            break
    return {
        "target_deg": math.degrees(target_rad), "achieved_deg": math.degrees(achieved),
        "peak_deg": math.degrees(peak), "err_deg": math.degrees(err),
        "overshoot_deg": math.degrees(overshoot), "undershoot_deg": math.degrees(undershoot),
        "backcreep_deg": math.degrees(backcreep),
        "settle_s": settle, "gz_rev": revs, "dur": ts[-1] - ts[0],
    }


def agg(legs):
    """Aggregate per-leg metrics into one outline summary (sample-weighted)."""
    legs = [m for m in legs if m]
    if not legs:
        return None
    tot = sum(m["n"] for m in legs)
    stot = sum(m["s_n"] for m in legs) or 1
    w = lambda k: sum(m[k] * m["n"] for m in legs) / tot
    ws = lambda k: sum(m[k] * m["s_n"] for m in legs) / stot
    return {
        "ct_rms": w("ct_rms"), "ct_peak": max(m["ct_peak"] for m in legs),
        "herr_rms": w("herr_rms"), "herr_peak": max(m["herr_peak"] for m in legs),
        "wz_zc": w("wz_zc"), "gz_zc": w("gz_zc"),
        "wz_peak": max(m["wz_peak"] for m in legs),
        "speed": w("speed"), "n": tot, "legs": len(legs),
        # straight regime
        "s_ct_rms": ws("s_ct_rms"), "s_ct_peak": max(m["s_ct_peak"] for m in legs),
        "s_herr_rms": ws("s_herr_rms"), "s_wz_zc": ws("s_wz_zc"),
        "s_gz_zc": ws("s_gz_zc"), "s_n": stot,
        # turn regime
        "t_ct_peak": max(m["t_ct_peak"] for m in legs),
        "t_herr_peak": max(m["t_herr_peak"] for m in legs),
        "t_wz_peak": max(m["t_wz_peak"] for m in legs),
    }


class PathCal(Node):
    def __init__(self):
        super().__init__("path_calibrate")
        self.sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odometry/filtered_map", self._odom, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, self.sub_qos)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self._cmd, 10)
        self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(HighLevelStatus, "/behavior_tree_node/high_level_status",
                                 self._hl, self.sub_qos)
        self.create_subscription(Emergency, HB + "/emergency", self._emg, self.sub_qos)
        self.create_subscription(Status, HB + "/status", self._status, self.sub_qos)

        self.cp = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.fp = ActionClient(self, FollowPath, "/follow_path")
        self.sp = ActionClient(self, Spin, "/spin")
        self.cov = ActionClient(self, ComputeCoveragePath, "/compute_coverage_path")
        self.hlc = self.create_client(HighLevelControl, "/behavior_tree_node/high_level_control")
        self.clr_g = self.create_client(ClearEntireCostmap,
                                        "/global_costmap/clear_entirely_global_costmap")
        self.clr_l = self.create_client(ClearEntireCostmap,
                                        "/local_costmap/clear_entirely_local_costmap")
        self.area_cli = self.create_client(GetMowingArea, "/map_server_node/get_mowing_area")
        self._set = {}
        self._get = {}

        self.x = self.y = self.yaw = self.sx = None
        self._vx = self._wz = self._gz = 0.0
        self._wheel_v = 0.0  # achieved forward speed from /wheel_odom (encoders)
        self.hl_state = None
        self.emergency = False
        self.is_charging = None
        self.recording = False
        self.samples = []
        self._fp_gh = None
        # high-rate gyro capture for the in-place yaw test (10 Hz pose can't
        # resolve a sub-second small pivot; IMU gyro is ~90 Hz).
        self._gz_buf = []
        self._capture_gyro = False

    # ---- callbacks -------------------------------------------------------
    def _odom(self, m):
        q = m.pose.pose.orientation
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.sx = math.sqrt(max(0.0, m.pose.covariance[0]))
        if self.recording:
            self.samples.append((time.time(), self.x, self.y, self.yaw,
                                 self._vx, self._wz, self._gz, self._wheel_v))

    def _on_wheel(self, m):
        self._wheel_v = m.twist.twist.linear.x  # encoder-derived forward speed

    def _cmd(self, m):
        self._vx = m.twist.linear.x
        self._wz = m.twist.angular.z

    def _imu(self, m):
        self._gz = m.angular_velocity.z
        if self._capture_gyro:
            self._gz_buf.append((time.time(), self._gz))

    def _hl(self, m):
        self.hl_state = m.state

    def _emg(self, m):
        self.emergency = bool(m.active_emergency or m.latched_emergency)

    def _status(self, m):
        self.is_charging = m.is_charging

    # ---- future helper (bg executor spins) -------------------------------
    def _await(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    # ---- params ----------------------------------------------------------
    def _set_cli(self, node):
        if node not in self._set:
            self._set[node] = self.create_client(SetParameters, node + "/set_parameters")
        return self._set[node]

    def _get_cli(self, node):
        if node not in self._get:
            self._get[node] = self.create_client(GetParameters, node + "/get_parameters")
        return self._get[node]

    def set_param(self, node, name, value, ptype):
        cli = self._set_cli(node)
        if not cli.wait_for_service(timeout_sec=3.0):
            return False
        p = ParamMsg(name=name)
        pv = ParameterValue(type=ptype)
        if ptype == ParameterType.PARAMETER_DOUBLE:
            pv.double_value = float(value)
        elif ptype == ParameterType.PARAMETER_BOOL:
            pv.bool_value = bool(value)
        p.value = pv
        req = SetParameters.Request(parameters=[p])
        res = self._await(cli.call_async(req), 3.0)
        return bool(res and res.results and res.results[0].successful)

    def get_params(self, node, names):
        cli = self._get_cli(node)
        out = {n: None for n in names}
        if not cli.wait_for_service(timeout_sec=3.0):
            return out
        res = self._await(cli.call_async(GetParameters.Request(names=names)), 3.0)
        if not res:
            return out
        for n, pv in zip(names, res.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                out[n] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_BOOL:
                out[n] = pv.bool_value
        return out

    # ---- BT mode + costmaps ---------------------------------------------
    def hl(self, cmd):
        if not self.hlc.wait_for_service(timeout_sec=5.0):
            return False
        req = HighLevelControl.Request()
        req.command = cmd
        return self._await(self.hlc.call_async(req), 5.0) is not None

    def clear_costmaps(self):
        for cl in (self.clr_g, self.clr_l):
            if cl.wait_for_service(timeout_sec=3.0):
                self._await(cl.call_async(ClearEntireCostmap.Request()), 3.0)

    def fetch_area(self, name):
        """Return [(x,y),...] vertices of the mowing area whose name matches,
        or (None, list_of_names) if not found."""
        if not self.area_cli.wait_for_service(timeout_sec=4.0):
            return None, []
        names = []
        for i in range(24):
            res = self._await(self.area_cli.call_async(GetMowingArea.Request(index=i)), 4.0)
            if not res or not res.success:
                break
            names.append(res.area.name)
            if res.area.name == name:
                pts = [(p.x, p.y) for p in res.area.area.points]
                return (pts if len(pts) >= 3 else None), names
        return None, names

    # ---- plan + follow ---------------------------------------------------
    def plan(self, gx, gy, gyaw):
        if not self.cp.wait_for_server(timeout_sec=6.0):
            return None
        g = ComputePathToPose.Goal()
        g.use_start = False
        g.planner_id = "GridBased"
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = gx, gy
        g.goal.pose.orientation.z, g.goal.pose.orientation.w = yq(gyaw)
        gh = self._await(self.cp.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            return None
        res = self._await(gh.get_result_async(), 15.0)
        if not res or not res.result.path.poses:
            return None
        return res.result.path

    def coverage_path(self, name, headland=0.5):
        """Ask the real F2C coverage server for the mowing path of an area
        (same goal the BT uses). Returns nav_msgs/Path or (None, names)."""
        verts, names = self.fetch_area(name)
        if not verts:
            return None, names
        if not self.cov.wait_for_server(timeout_sec=8.0):
            return None, names
        ring = Coordinates()
        ring.coordinates = [Coordinate(axis1=float(x), axis2=float(y)) for (x, y) in verts]
        if verts[0] != verts[-1]:  # close the ring (F2C expects closed)
            ring.coordinates.append(Coordinate(axis1=float(verts[0][0]), axis2=float(verts[0][1])))
        g = ComputeCoveragePath.Goal()
        g.generate_headland = g.generate_route = g.generate_path = True
        g.use_gml_file = False
        g.frame_id = "map"
        g.polygons = [ring]
        g.headland_mode.mode = "CONSTANT"
        g.headland_mode.width = float(headland)
        g.swath_mode.objective = "LENGTH"
        g.swath_mode.mode = "BRUTE_FORCE"
        g.swath_mode.best_angle = 0.0
        g.swath_mode.step_angle = math.pi / 180.0
        g.route_mode.mode = "BOUSTROPHEDON"
        g.path_mode.mode = "DUBIN"
        g.path_mode.continuity_mode = "DISCONTINUOUS"
        g.path_mode.turn_point_distance = 0.05
        gh = self._await(self.cov.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            return None, names
        res = self._await(gh.get_result_async(), 30.0)
        if not res or not res.result.nav_path.poses:
            return None, names
        return res.result.nav_path, names

    def straight_path(self, ax, ay, bx, by, yaw, step=0.05):
        """A straight, densified Path from (ax,ay) to (bx,by), every pose at
        `yaw`. This is the literal edge of the polygon — the reference for
        measuring straight-line following (vs Smac, which inserts turn arcs)."""
        d = math.hypot(bx - ax, by - ay)
        n = max(1, int(d / step))
        qz, qw = yq(yaw)
        p = Path()
        p.header.frame_id = "map"
        for k in range(n + 1):
            t = k / n
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = ax + (bx - ax) * t
            ps.pose.position.y = ay + (by - ay) * t
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            p.poses.append(ps)
        return p

    def follow(self, path, record=True, timeout=180.0,
               controller_id="FollowPath", goal_checker_id="stopped_goal_checker"):
        if not self.fp.wait_for_server(timeout_sec=6.0):
            return "NO_SRV"
        g = FollowPath.Goal()
        g.path = path
        g.controller_id = controller_id
        g.goal_checker_id = goal_checker_id
        if record:
            self.samples = []
            self.recording = True
        gh = self._await(self.fp.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            self.recording = False
            return "REJECTED"
        self._fp_gh = gh
        rf = gh.get_result_async()
        res = self._await(rf, timeout)
        self.recording = False
        self._fp_gh = None
        if not res:
            return "TIMEOUT"
        return {4: "OK", 5: "CANCEL", 6: "ABORT"}.get(res.status, str(res.status))

    def cancel_follow(self):
        if self._fp_gh is not None:
            try:
                self._await(self._fp_gh.cancel_goal_async(), 3.0)
            except Exception:
                pass

    def spin(self, target_rad, record=True, timeout=40.0):
        if not self.sp.wait_for_server(timeout_sec=6.0):
            return "NO_SRV"
        g = Spin.Goal()
        g.target_yaw = float(target_rad)
        g.time_allowance = Duration(sec=int(timeout))
        if record:
            self.samples = []
            self.recording = True
            self._gz_buf = []
            self._capture_gyro = True
        gh = self._await(self.sp.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            self.recording = False
            self._capture_gyro = False
            return "REJECTED"
        self._fp_gh = gh
        res = self._await(gh.get_result_async(), timeout + 10.0)
        # Keep capturing ~0.9 s after the spin to record any coast-back creep:
        # at zero command the motors go to zero PWM (no holding torque), so
        # drivetrain backlash + tire/gearbox windup can unwind in reverse.
        time.sleep(0.9)
        self.recording = False
        self._capture_gyro = False
        self._fp_gh = None
        if not res:
            return "TIMEOUT"
        return {4: "OK", 5: "CANCEL", 6: "ABORT"}.get(res.status, str(res.status))


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        return "q"


def fmt_d(v, unit=""):
    if v is None:
        return c("?", Col.DIM)
    if isinstance(v, float) and math.isnan(v):
        return c("default", Col.DIM)
    return c(f"{v:.2f}{unit}", Col.CYN)


def read_cfg(node):
    cfg = node.get_params(HB, ["ff_coulomb_byte", "ff_viscous_byte_per_mps",
                               "ff_breakaway_byte", "angular_rate_loop_enabled"])
    cfg.update(node.get_params(CS, ["FollowPath.kd_lat", "FollowPath.derivative_filter_tau",
                                    "FollowPath.speed_fast"]))
    return cfg


def cruise_cmd(node):
    """The speed FollowPath actually commands (FTC speed_fast), default 0.4."""
    v = node.get_params(CS, ["FollowPath.speed_fast"]).get("FollowPath.speed_fast")
    return v if isinstance(v, (int, float)) and v > 0 else 0.4


def header(node, cfg, geo):
    clear()
    bar = c("=" * 68, Col.DIM)
    print(bar)
    print(c("  Path-Following Calibration Assistant", Col.B + Col.MAG))
    print(bar)
    st = node.hl_state
    name = HL_NAMES.get(st, "?")
    mode = (c(f"{name} (drives)", Col.GRN) if st in HL_DRIVES
            else c(f"{name} -> set RECORDING (menu p) so Nav2 can drive", Col.RED))
    print(f"  BT mode    : {mode}")
    if node.is_charging:
        print(f"  charging   : {c('ON DOCK -- undock first', Col.RED)}")
    sig = node.sx
    if sig is None:
        sg = c("no localization", Col.RED)
    elif sig > 0.30:
        sg = c(f"{sig:.2f} m  DEGRADED (don't trust metrics)", Col.RED)
    elif sig > 0.10:
        sg = c(f"{sig:.2f} m  marginal", Col.YEL)
    else:
        sg = c(f"{sig:.2f} m  good", Col.GRN)
    print(f"  sigma_xy   : {sg}")
    print(f"  emergency  : " + (c('ACTIVE', Col.RED) if node.emergency else c('clear', Col.GRN)))
    print(bar)
    print("  Feedforward (firmware)   FTC anti-hunt (controller)")
    print(f"    fc = {fmt_d(cfg.get('ff_coulomb_byte'))}   "
          f"fv = {fmt_d(cfg.get('ff_viscous_byte_per_mps'))}   "
          f"fb = {fmt_d(cfg.get('ff_breakaway_byte'))}")
    gyro = cfg.get("angular_rate_loop_enabled")
    gtxt = c("ON", Col.YEL) if gyro else c("off", Col.DIM) if gyro is not None else c("?", Col.DIM)
    print(f"    kd_lat = {fmt_d(cfg.get('FollowPath.kd_lat'))}   "
          f"deriv_LPF = {fmt_d(cfg.get('FollowPath.derivative_filter_tau'), 's')}   "
          f"gyro_loop = {gtxt}")
    sf = cfg.get("FollowPath.speed_fast")
    sftxt = fmt_d(sf, " m/s")
    note = c("  (mowing is 0.5 -- raise via 'v' to test at mowing speed)", Col.DIM) \
        if (isinstance(sf, (int, float)) and sf < 0.49) else ""
    print(f"    test cruise (FollowPath.speed_fast) = {sftxt}{note}")
    if geo["mode"] == "area":
        ins = f"  inset {geo['inset']:.1f}m" if geo["inset"] > 0 else "  (raw boundary)"
        src = c(f"area '{geo['name']}'{ins}", Col.CYN)
    else:
        src = c(f"rectangle {geo['a']:.1f} x {geo['b']:.1f} m", Col.CYN)
    edges = geo.get("edges", "straight")
    em = c("straight edges", Col.GRN) if edges == "straight" else c("smac (curvy)", Col.YEL)
    print(f"  outline    : {src}  |  {em}")
    print(bar)


def vcol(val, good, marg):
    """ANSI colour for a lower-is-better metric."""
    return Col.GRN if val <= good else (Col.YEL if val <= marg else Col.RED)


def mv(label, val, unit, good, marg):
    """'label=VALUE' with VALUE coloured good/marginal/poor."""
    return f"{label}=" + c(f"{val:.1f}{unit}", vcol(val, good, marg))


def print_legend(node):
    cmd = cruise_cmd(node)
    g, y, r = c("good", Col.GRN), c("marginal", Col.YEL), c("poor", Col.RED)
    print(c("  --- metrics (lower is better unless noted) ---", Col.DIM))
    print(f"    ct     cross-track from planned path [cm]   straight: {g} <5 / {y} 5-10 / {r} >10")
    print(f"    ctpk   cross-track PEAK on this section [cm] turn:     {g} <15 / {y} 15-30 / {r} >30")
    print(f"    hunt   steering oscillation [zero-cross/s]            {g} <1.5 / {y} 1.5-3 / {r} >3")
    print(c(f"    herrpk heading-error PEAK [deg] -- ~30-45 at SHARP corners is normal (the robot "
            "rounds the vertex); judge it against the straights, not in absolute.", Col.DIM))
    print(f"    v      achieved cruise speed [m/s]  -- want >= {0.9*cmd:.2f} (90% of commanded {cmd:.2f})")
    print(c("  " + "-" * 60, Col.DIM))


def local_to_map(node, lx, ly):
    cy, sy = math.cos(node.yaw), math.sin(node.yaw)
    return node.x + lx * cy - ly * sy, node.y + lx * sy + ly * cy


def run_outline(node, direction, geo):
    """Drive an outline CW or CCW as a sequence of Smac legs. Source is either
    the real 'area' polygon (transit to it, then follow its edges) or a synthetic
    'rect' anchored at the robot's current pose."""
    if node.x is None:
        print(c("  no pose yet", Col.RED))
        return None
    if geo["mode"] == "area":
        verts, names = node.fetch_area(geo["name"])
        if not verts:
            print(c(f"  area {geo['name']!r} not found. Available: {names}", Col.RED))
            return None
        pts = order_loop(verts, (node.x, node.y), direction, geo["inset"])
        # transit (un-recorded) to the start vertex, aligned to the first edge
        v0, v1 = pts[0], pts[1]
        print(c(f"\n  transit to start vertex ({v0[0]:.2f},{v0[1]:.2f}) ...", Col.DIM))
        tp = node.plan(v0[0], v0[1], math.atan2(v1[1] - v0[1], v1[0] - v0[0]))
        if tp is None:
            print(c("  transit PLAN_FAIL (try a small inset, or clear costmaps with 'p')", Col.RED))
            return None
        node.follow(tp, record=False)
        time.sleep(0.6)
    else:
        a, b = geo["a"], geo["b"]
        # local corners, both start with a forward leg; ccw turns left, cw turns right
        if direction == "ccw":
            local = [(0, 0), (a, 0), (a, b), (0, b), (0, 0)]
        else:
            local = [(0, 0), (a, 0), (a, -b), (0, -b), (0, 0)]
        pts = [local_to_map(node, lx, ly) for (lx, ly) in local]
    leg_metrics = []
    print(c(f"\n  === OUTLINE {direction.upper()} ({len(pts)-1} legs) ===", Col.B + Col.BLU))
    print_legend(node)
    cmd_v = cruise_cmd(node)
    straight = geo.get("edges", "straight") == "straight"
    for i in range(len(pts) - 1):
        gx, gy = pts[i + 1]
        gyaw = math.atan2(gy - pts[i][1], gx - pts[i][0])
        kind = "straight" if straight else "plan"
        sys.stdout.write(f"  leg {i+1}: {kind} -> ({gx:.2f},{gy:.2f}) ... ")
        sys.stdout.flush()
        if straight:
            # Start the segment from the robot's ACTUAL pose (it stops a bit short
            # of each vertex), not the ideal vertex -- otherwise the path begins
            # off-axis ahead of the robot and FTC loops to reach it. Drive straight
            # from where we are to the next vertex (corners cut slightly, which is
            # fine and realistic).
            sx, sy = node.x, node.y
            path = node.straight_path(sx, sy, gx, gy, math.atan2(gy - sy, gx - sx))
        else:
            path = node.plan(gx, gy, gyaw)
        if path is None:
            print(c("PLAN_FAIL (skipping)", Col.RED))
            continue
        poly = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
        st = node.follow(path, record=True)
        m = analyze(node.samples, poly)
        leg_metrics.append(m)
        if m:
            v = m["speed"]
            vc = Col.GRN if v >= 0.9 * cmd_v else (Col.YEL if v >= 0.7 * cmd_v else Col.RED)
            stc = c(st, Col.GRN if st == "OK" else Col.YEL)
            print(f"  {stc}  " + mv("straight ct", m["s_ct_rms"] * 100, "cm", 5, 10) + " "
                  + mv("hunt", m["s_wz_zc"], "/s", 1.5, 3) + " | "
                  + mv("turn ctpk", m["t_ct_peak"] * 100, "cm", 15, 30)
                  + f"  herrpk={m['t_herr_peak']:.0f}deg | v=" + c(f"{v:.2f}", vc))
        else:
            print(c(f"{st}  (no usable samples)", Col.YEL))
        if node.emergency:
            print(c("  EMERGENCY -- aborting outline", Col.RED))
            break
        time.sleep(0.5)
    result = agg(leg_metrics)
    if result:
        result["attempted"] = len(pts) - 1
    return result


def run_coverage(node, geo):
    """Drive the REAL F2C coverage path (swaths + Dubins arcs) via the coverage
    controller -- the production mowing path, not a synthetic outline."""
    print(c("\n  === COVERAGE PATH TEST (real F2C mowing path) ===", Col.B + Col.BLU))
    if node.hl_state not in HL_DRIVES:
        print(c("  not in a driving mode -- press 'p' first.", Col.RED))
        return None
    print(c(f"  requesting coverage path for area '{geo['name']}' ...", Col.DIM))
    cov, names = node.coverage_path(geo["name"], geo.get("headland", 0.5))
    if cov is None:
        print(c(f"  coverage planning FAILED (area {geo['name']!r}? available: {names})", Col.RED))
        return None
    poly = [(p.pose.position.x, p.pose.position.y) for p in cov.poses]
    print(c(f"  coverage path: {len(poly)} poses; transit to start ...", Col.DIM))
    if len(poly) >= 2:
        s0, s1 = poly[0], poly[1]
        tp = node.plan(s0[0], s0[1], math.atan2(s1[1] - s0[1], s1[0] - s0[0]))
        if tp is not None:
            node.follow(tp, record=False)
            time.sleep(0.6)
    print_legend(node)
    cmd_v = cruise_cmd(node)
    st = node.follow(cov, record=True, timeout=600.0,
                     controller_id="FollowCoveragePath", goal_checker_id="coverage_goal_checker")
    m = analyze(node.samples, poly)
    if m:
        v = m["speed"]
        vc = Col.GRN if v >= 0.9 * cmd_v else (Col.YEL if v >= 0.7 * cmd_v else Col.RED)
        stc = c(st, Col.GRN if st == "OK" else Col.YEL)
        print(f"  {stc}  swaths: " + mv("ct", m["s_ct_rms"] * 100, "cm", 5, 10) + " "
              + mv("hunt", m["s_wz_zc"], "/s", 1.5, 3) + " | arcs: "
              + mv("ctpk", m["t_ct_peak"] * 100, "cm", 15, 30)
              + f"  herrpk={m['t_herr_peak']:.0f}deg | v=" + c(f"{v:.2f}", vc))
        for ln in suggest(node, m, None):
            print(ln)
    else:
        print(c(f"  {st}  (no usable samples)", Col.YEL))
    return m


def show_summary(node, cw, ccw):
    print(c("\n  === SUMMARY ===", Col.B))
    hdr = f"  {'metric':<22}{'CW':>12}{'CCW':>12}"
    print(c(hdr, Col.B))
    print(c("  " + "-" * 44, Col.DIM))

    cmd_v = cruise_cmd(node)

    def row(label, key, scale=1.0, unit="", good=None, marg=None, higher=False):
        def cell(agg):
            if not agg:
                return f"{'-':>12}"
            v = agg[key] * scale
            padded = f"{v:.1f}{unit}".rjust(12)
            if good is None:
                return padded
            if higher:
                col = Col.GRN if v >= good else (Col.YEL if v >= marg else Col.RED)
            else:
                col = vcol(v, good, marg)
            return c(padded, col)
        print(f"  {label:<22}{cell(cw)}{cell(ccw)}")

    if cw or ccw:
        print(c("  STRAIGHTS", Col.B))
        row("  cross-track RMS", "s_ct_rms", 100, "cm", 5, 10)
        row("  cross-track PEAK", "s_ct_peak", 100, "cm", 10, 20)
        row("  hunt (zc/s)", "s_wz_zc", 1, "", 1.5, 3)
        print(c("  TURNS", Col.B))
        row("  cross-track PEAK", "t_ct_peak", 100, "cm", 15, 30)
        row("  heading PEAK", "t_herr_peak", 1, "deg")  # uncoloured: high at sharp corners
        print(c("  OVERALL", Col.B))
        row("  cruise speed", "speed", 1, "m/s", 0.9 * cmd_v, 0.7 * cmd_v, higher=True)
        print(c(f"  (cruise target {cmd_v:.2f} m/s; heading PEAK is high at sharp corners -- "
                "normal)", Col.DIM))
    return suggest(node, cw, ccw)


def suggest(node, cw, ccw):
    """Prioritized next-adjustment proposals from the metrics."""
    out = [c("\n  === PROPOSAL ===", Col.B)]
    runs = [m for m in (cw, ccw) if m]
    if not runs:
        out.append(c("  no usable data -- check mode/localization and retry.", Col.RED))
        return out
    s_ct = max(m["s_ct_rms"] for m in runs)     # STRAIGHT tracking
    t_ctpk = max(m["t_ct_peak"] for m in runs)   # TURN overshoot
    s_wz = max(m["s_wz_zc"] for m in runs)       # STRAIGHT hunting (command)
    s_gz = max(m["s_gz_zc"] for m in runs)       # STRAIGHT hunting (gyro)
    spd = min(m["speed"] for m in runs)
    herr = max(m["herr_rms"] for m in runs)

    tips = []
    # hunting on the STRAIGHTS only (turning wz is legitimate, excluded)
    if s_wz > 2.0 or s_gz > 2.0:
        if s_gz > s_wz + 0.5:
            tips.append(("HUNT (wheels, straight)", Col.RED,
                         "gyro oscillates more than the command on straights -> motor stall/slip. "
                         "Raise fc a few byte (and keep fb>fc for re-stuck recovery)."))
        else:
            tips.append(("HUNT (controller, straight)", Col.RED,
                         "steering oscillates on the straights -> LOWER FollowPath.kd_lat (or raise "
                         "derivative_filter_tau toward 0.3). If gyro_loop is ON, try turning it off."))
    # under-speed: fv too low. Floor = 85% of what FollowPath commands.
    cmd_v = cruise_cmd(node)
    if spd < 0.85 * cmd_v:
        tips.append(("UNDER-SPEED", Col.YEL,
                     f"cruise {spd:.2f} m/s vs commanded {cmd_v:.2f} m/s -> raise fv until the "
                     "straights reach the commanded speed (set speed_fast to 0.5 with 'v' to tune "
                     "at mowing speed)."))
    # cross-track at the TURNS
    if t_ctpk > 0.30:
        tips.append(("TURN OVERSHOOT", Col.YEL,
                     "peak cross-track at the corners -> raise fc/fb margin for clean pivots, relax "
                     "turn rate (kp_ang), or check minimum_turning_radius. Use the 'z' yaw test to "
                     "isolate pivot quality."))
    # cross-track on the STRAIGHTS
    if s_ct > 0.10:
        tips.append(("STRAIGHT TRACKING", Col.YEL,
                     "cross-track RMS is high on the straights -> raise FollowPath.kp_lat, or "
                     "increase max_follow_distance (lookahead) for less weave."))
    if herr > 15:
        tips.append(("HEADING", Col.YEL,
                     "heading error is high -> kp_ang / gyro-loop region; tune after cross-track."))
    # directional asymmetry CW vs CCW (straight tracking)
    if cw and ccw:
        if abs(cw["s_ct_rms"] - ccw["s_ct_rms"]) > 0.05:
            worse = "CW" if cw["s_ct_rms"] > ccw["s_ct_rms"] else "CCW"
            tips.append(("L/R ASYMMETRY", Col.MAG,
                         f"{worse} tracks worse -> directional FF/wheel asymmetry or a gyro/heading "
                         "bias. Re-check L/R symmetry (ff_calibrate rotate L vs R, or the 'z' yaw "
                         "test +/- angles)."))

    if not tips:
        out.append(c("  Looks good: low cross-track, low hunting, balanced CW/CCW. "
                     "Tighten goal tolerances or raise speed to stress it further.", Col.GRN))
    else:
        for i, (tag, col, txt) in enumerate(tips, 1):
            out.append(c(f"  {i}. [{tag}] ", Col.B + col) + txt)
    return out


# --------------------------------------------------------------------------
# Rotation (yaw-to-target) test -- the dock-orientation case
# --------------------------------------------------------------------------
def _agg_reps(angle, reps_m):
    """Aggregate N repetitions of one target angle into success-rate stats."""
    tol = max(1.0, 0.4 * abs(angle))         # "landed on target" tolerance (deg)
    movethr = max(0.2, 0.3 * abs(angle))     # "actually moved" threshold (deg)
    overthr = max(1.0, 0.5 * abs(angle))     # "overshot" threshold (deg)
    creepthr = max(0.5, 0.2 * abs(angle))    # "crept back" threshold (deg)
    sgn = 1.0 if angle >= 0 else -1.0
    tgt = abs(angle)
    ach, peak, over, under, creep, abserr, settle = [], [], [], [], [], [], []
    ok = no_move = n_under = n_over = n_creep = 0
    osc = 0
    for st, m in reps_m:
        if not m:
            continue
        ach.append(m["achieved_deg"])
        peak.append(m["peak_deg"])
        over.append(m["overshoot_deg"])
        under.append(m["undershoot_deg"])
        creep.append(m["backcreep_deg"])
        abserr.append(abs(m["err_deg"]))
        settle.append(m["settle_s"])
        osc = max(osc, m["gz_rev"])
        peak_toward = m["peak_deg"] * sgn
        moved = peak_toward > movethr
        reached = peak_toward >= tgt - tol     # the pivot DID reach target at its peak
        if not moved:
            no_move += 1                        # never broke free
        elif not reached:
            n_under += 1                        # deadband: peak never reached target
        if m["backcreep_deg"] > creepthr:
            n_creep += 1                        # reached/moved then crept back at rest
        if m["overshoot_deg"] > overthr:
            n_over += 1
        if st == "OK" and moved and abs(m["err_deg"]) <= tol:
            ok += 1
    avg = lambda L: sum(L) / len(L) if L else 0.0
    measured = sum(1 for st, m in reps_m if m)
    return {"angle": angle, "reps": measured, "no_data": len(reps_m) - measured,
            "ok": ok, "ach": avg(ach), "peak": avg(peak), "over": avg(over),
            "under": avg(under), "creep": avg(creep), "abserr": avg(abserr),
            "settle": avg(settle), "no_move": no_move, "n_under": n_under,
            "n_over": n_over, "n_creep": n_creep, "osc": osc}


def run_rotation(node, angles, reps=1):
    label = "FINE-YAW" if max(abs(a) for a in angles) <= 5 else "ROTATION"
    print(c(f"\n  === {label} TEST (in-place yaw-to-target, {reps} rep(s)) ===", Col.B + Col.BLU))
    if node.hl_state not in HL_DRIVES:
        print(c("  not in a driving mode -- press 'p' first.", Col.RED))
        return
    print(c("  simulates orienting in place (e.g. squaring up to the dock). +ve = CCW.", Col.DIM))
    aggs = []
    for ang in angles:
        reps_m = []
        for r in range(reps):
            if node.emergency:
                print(c("  EMERGENCY -- aborting", Col.RED))
                break
            sys.stdout.write(f"  {ang:+.0f} deg (rep {r+1}/{reps}) ... ")
            sys.stdout.flush()
            # stationary gyro baseline (bias) over ~0.4 s before the spin
            gb, t0 = [], time.time()
            while time.time() - t0 < 0.4:
                gb.append(node._gz)
                time.sleep(0.02)
            baseline = sum(gb) / len(gb) if gb else 0.0
            st = node.spin(math.radians(ang))
            time.sleep(0.3)
            m = analyze_spin(node._gz_buf, math.radians(ang), baseline)
            reps_m.append((st, m))
            if m:
                print(c(f"{st}: peak {m['peak_deg']:+.1f} -> rest {m['achieved_deg']:+.1f}  "
                        f"creep {m['backcreep_deg']:.1f}  over {m['overshoot_deg']:.1f}", Col.DIM))
            else:
                print(c(f"{st}: NO IMU DATA", Col.YEL))
            time.sleep(0.5)
        if not reps_m:
            continue
        a = _agg_reps(ang, reps_m)
        aggs.append(a)
        col = Col.GRN if (a["reps"] and a["ok"] == a["reps"]) else \
            (Col.YEL if a["ok"] > 0 else Col.RED)
        nodata = f", {a['no_data']} no-data" if a["no_data"] else ""
        print(c(f"   => {ang:+.0f}deg: {a['ok']}/{a['reps']} ok  peak {a['peak']:+.1f} -> rest "
                f"{a['ach']:+.1f}  creep {a['creep']:.1f}  err {a['abserr']:.1f}deg  "
                f"({a['no_move']} no-move, {a['n_under']} short, {a['n_over']} over, "
                f"{a['n_creep']} creep{nodata})", col))
        if node.emergency:
            break
    for ln in rotation_suggest(aggs):
        print(ln)


def rotation_suggest(aggs):
    out = [c("\n  === YAW PROPOSAL ===", Col.B)]
    v = [a for a in aggs if a["reps"] > 0]
    if not v:
        out.append(c("  no usable rotations.", Col.RED))
        return out
    tot_nomove = sum(a["no_move"] for a in v)
    max_over = max(a["over"] for a in v)
    n_over = sum(a["n_over"] for a in v)
    n_under = sum(a["n_under"] for a in v)
    max_under = max(a["under"] for a in v)
    n_creep = sum(a["n_creep"] for a in v)
    max_creep = max(a["creep"] for a in v)
    max_osc = max(a["osc"] for a in v)
    # smallest angle that landed on target on EVERY rep = the fine-adjustment floor
    reliable = sorted(abs(a["angle"]) for a in v if a["ok"] == a["reps"])
    floor = reliable[0] if reliable else None

    if floor is not None:
        out.append(c(f"  Smallest reliably-achieved adjustment: {floor:.0f} deg.",
                     Col.GRN if floor <= 3 else Col.YEL))
    else:
        out.append(c("  No target angle was hit reliably on every rep.", Col.RED))

    tips = []
    if tot_nomove > 0:
        small = [a for a in v if a["no_move"] > 0]
        sm_ang = min(abs(a["angle"]) for a in small)
        tips.append(("DEADBAND / NO-MOVE", Col.RED,
                     f"some commands (from {sm_ang:.0f}deg) produced no motion -> the brief small "
                     "command can't break static friction open-loop. To go finer: raise fb (start "
                     "kick) or raise the spin approach speed (behavior_server Spin min_rotational_vel) "
                     "-- both trade against overshoot. Otherwise ~the floor above is your practical "
                     "minimum."))
    if n_creep > 0:
        tips.append(("BACK-CREEP / HOLDING TORQUE", Col.RED,
                     f"reaches target then creeps back ~{max_creep:.1f}deg at rest -> at zero command "
                     "the motors coast (no holding torque) and drivetrain backlash + tire windup "
                     "unwind. This is FIRMWARE/mechanical, NOT FF tuning: it needs an active hold/brake "
                     "at zero command or a position-hold loop. The dock cradle's mechanical capture "
                     "mitigates it. The FINAL resting yaw (not the peak) is what lands at the dock."))
    if n_under > 0:
        tips.append(("UNDERSHOOT (deadband)", Col.YEL,
                     f"moved but stalled short of target (up to {max_under:.1f}deg) -> the pivot "
                     "decelerates into the deadband and stops before the goal. RAISE the spin "
                     "approach/min speed (behavior_server Spin min_rotational_vel) so it stays above "
                     "the deadband to target; lowering fc also helps it creep the last bit."))
    if n_over > 0 or max_over > 3:
        tips.append(("OVERSHOOT", Col.YEL,
                     f"overshoots up to {max_over:.1f}deg -> too much speed/momentum at the target. "
                     "REDUCE the spin approach/max speed and/or lower fc (gentler, less momentum); "
                     "keep fb just above fc."))
    if n_under > 0 and (n_over > 0 or max_over > 3):
        tips.append(("UNDER vs OVER", Col.CYN,
                     "both short and past in the same set -> the APPROACH SPEED is the opposite-"
                     "trade-off knob (faster cures undershoot but adds overshoot). Tune it to the "
                     "value that just reaches target without going past."))
    if max_osc > 2:
        tips.append(("OSCILLATION", Col.RED,
                     "the pivot hunts before settling -> fc too low (stall-slip; raise fc/keep fb>fc) "
                     "or rotation gain too high."))
    # direction asymmetry: compare achieved at +a vs -a of equal magnitude
    mags = {}
    for a in v:
        mags.setdefault(abs(a["angle"]), {})[a["angle"] >= 0] = a["ach"]
    for mag, d in mags.items():
        if True in d and False in d and abs(d[True] + d[False]) > max(1.0, 0.3 * mag):
            tips.append(("L/R ASYMMETRY", Col.MAG,
                         f"+{mag:.0f} and -{mag:.0f} achieve unequal magnitudes -> left/right drive "
                         "asymmetry (re-check ff_calibrate rotate L vs R)."))
            break
    if not tips:
        out.append(c("  Clean fine control: reliable, on-target, no overshoot/hunt.", Col.GRN))
    else:
        for i, (tag, col, txt) in enumerate(tips, 1):
            out.append(c(f"  {i}. [{tag}] ", Col.B + col) + txt)
    return out


# --------------------------------------------------------------------------
# Auto-test (parameter sweep)
# --------------------------------------------------------------------------
# key -> (node, param-name, type). The knobs the sweep can vary.
PARAM_REG = {
    "fc": (HB, "ff_coulomb_byte", "double"),
    "fb": (HB, "ff_breakaway_byte", "double"),
    "fv": (HB, "ff_viscous_byte_per_mps", "double"),
    "kd_lat": (CS, "FollowPath.kd_lat", "double"),
    "gyro": (HB, "angular_rate_loop_enabled", "bool"),
}
SWEEP_ORDER = ["fc", "fb", "fv", "kd_lat", "gyro"]


def apply_param(node, key, val):
    nd, pn, ty = PARAM_REG[key]
    pt = ParameterType.PARAMETER_BOOL if ty == "bool" else ParameterType.PARAMETER_DOUBLE
    return node.set_param(nd, pn, val, pt)


def _num(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)) and not (isinstance(v, float) and math.isnan(v)):
        return v
    return None


def score(m, w):
    """Lower is better. Penalizes cross-track, heading, hunting, under-speed,
    and missing/aborted legs. None metrics (failed run) = huge penalty."""
    if not m:
        return 1e6
    s = (w["ct_rms"] * m.get("s_ct_rms", m["ct_rms"]) * 100       # straight tracking
         + w["ct_peak"] * m.get("t_ct_peak", m["ct_peak"]) * 100   # turn overshoot
         + w["herr"] * m["herr_rms"]
         + w["wz"] * m.get("s_wz_zc", m["wz_zc"])                  # straight hunting only
         + w["gz"] * m.get("s_gz_zc", m["gz_zc"])
         + w["uspd"] * max(0.0, w["spd_target"] - m["speed"]) * 100)
    miss = m.get("attempted", m["legs"]) - m["legs"]
    return s + w["miss"] * miss


def _combo_str(combo):
    return " ".join(f"{k}={v}" for k, v in combo.items())


def run_autotest(node, geo, sweep, weights):
    import itertools
    import json

    keys = [k for k in SWEEP_ORDER if k in sweep["enabled"]]
    if not keys:
        print(c("  no parameters enabled for the sweep (menu x to configure)", Col.YEL))
        time.sleep(1.5)
        return
    mode, direction = sweep["mode"], sweep["direction"]
    # Under-speed floor = 90% of what FollowPath actually commands (adaptive, so
    # it is correct whether you test at 0.4 or raise speed_fast to 0.5).
    weights = dict(weights)
    weights["spd_target"] = 0.9 * cruise_cmd(node)
    if mode == "grid":
        n = 1
        for k in keys:
            n *= len(sweep["params"][k])
    else:
        n = sum(len(sweep["params"][k]) for k in keys)

    header(node, read_cfg(node), geo)
    print(c(f"  AUTO-TEST: {mode} over {keys}  dir={direction}", Col.B + Col.BLU))
    print(c(f"  ~{n} outline runs (each = transit + perimeter, a few min). "
            "Battery + supervision!", Col.YEL))
    if node.hl_state not in HL_DRIVES:
        print(c("  not in a driving mode -- press 'p' first. Aborting.", Col.RED))
        time.sleep(2.0)
        return
    if ask("  type 'go' to start: ").lower() != "go":
        return

    ts = time.strftime("%Y%m%d-%H%M%S")
    logpath = f"/ros2_ws/maps/path_sweep_{ts}.jsonl"
    results = []
    log = open(logpath, "w")
    log.write(json.dumps({"meta": {"mode": mode, "dir": direction, "keys": keys,
                                   "params": sweep["params"], "weights": weights,
                                   "outline": geo}}) + "\n")
    log.flush()

    def evaluate(combo):
        for k, v in combo.items():
            apply_param(node, k, v)
        time.sleep(0.6)
        m = run_outline(node, direction, geo)
        sc = score(m, weights)
        results.append((sc, dict(combo), m))
        rec = {"combo": combo, "score": sc, "metrics": m}
        log.write(json.dumps(rec) + "\n")
        log.flush()
        tag = (f"ct={m['ct_rms']*100:.1f}cm pk={m['ct_peak']*100:.1f} "
               f"wz={m['wz_zc']:.1f} v={m['speed']:.2f}") if m else "FAILED"
        print(c(f"  [{len(results)}/{n}] {_combo_str(combo):<34} score={sc:7.1f}  {tag}",
                Col.GRN if (m and sc < 1e5) else Col.YEL))
        return sc

    best_combo = None
    try:
        if mode == "grid":
            for vals in itertools.product(*[sweep["params"][k] for k in keys]):
                if node.emergency:
                    print(c("  EMERGENCY -- aborting sweep", Col.RED))
                    break
                evaluate(dict(zip(keys, vals)))
        else:  # coordinate descent
            cfg = read_cfg(node)
            best = {"fc": _num(cfg.get("ff_coulomb_byte")),
                    "fb": _num(cfg.get("ff_breakaway_byte")),
                    "fv": _num(cfg.get("ff_viscous_byte_per_mps")),
                    "kd_lat": _num(cfg.get("FollowPath.kd_lat")),
                    "gyro": cfg.get("angular_rate_loop_enabled")}
            for k in keys:
                local = (None, 1e9)
                for v in sweep["params"][k]:
                    if node.emergency:
                        break
                    sc = evaluate({k: v})  # other knobs stay at the locked best
                    if sc < local[1]:
                        local = (v, sc)
                if local[0] is not None:
                    apply_param(node, k, local[0])
                    best[k] = local[0]
                    print(c(f"  -> locked {k} = {local[0]} (best so far)", Col.CYN))
                if node.emergency:
                    break
            best_combo = {k: best[k] for k in keys if best[k] is not None}
    except KeyboardInterrupt:
        print(c("\n  interrupted -- stopping sweep, results saved", Col.YEL))
        node.cancel_follow()
    finally:
        log.close()

    if not results:
        print(c("  no results", Col.RED))
        return
    results.sort(key=lambda r: r[0])
    print(c("\n  === RANKED (best first) ===", Col.B))
    for sc, combo, m in results[:8]:
        tag = (f"ct_rms={m['ct_rms']*100:.1f}cm peak={m['ct_peak']*100:.1f} "
               f"herr={m['herr_rms']:.0f} wz_zc={m['wz_zc']:.1f} v={m['speed']:.2f}"
               if m else "FAILED")
        print(f"  score {sc:7.1f}  {_combo_str(combo):<34} {tag}")
    if mode == "grid":
        best_combo = results[0][1]
    print(c(f"\n  BEST: {_combo_str(best_combo)}" if best_combo else "  no best", Col.B + Col.GRN))
    print(c(f"  log: {logpath}", Col.DIM))
    if best_combo and mode == "grid":
        if ask("  apply best combo now? [Y/n]: ").lower()[:1] != "n":
            for k, v in best_combo.items():
                apply_param(node, k, v)
            print(c("  applied.", Col.GRN))
    elif best_combo:
        print(c("  (descent already left the best values applied)", Col.DIM))
    ask(c("\n  ENTER...", Col.DIM))


def autotest_setup(node, geo, sweep, weights):
    while True:
        clear()
        print(c("  Auto-test setup", Col.B + Col.MAG))
        print(c("  " + "-" * 40, Col.DIM))
        for k in SWEEP_ORDER:
            if k == "gyro":
                continue
            en = k in sweep["enabled"]
            vals = ",".join(str(x) for x in sweep["params"][k])
            mark = c("ON ", Col.GRN) if en else c("off", Col.DIM)
            print(f"   {k:<7} [{mark}]  {vals}")
        n = (1 if sweep["mode"] == "grid" else 0)
        keys = [k for k in SWEEP_ORDER if k in sweep["enabled"]]
        if sweep["mode"] == "grid":
            for k in keys:
                n *= len(sweep["params"][k])
        else:
            n = sum(len(sweep["params"][k]) for k in keys)
        print(c("  " + "-" * 40, Col.DIM))
        print(f"  mode={c(sweep['mode'], Col.CYN)}  dir={c(sweep['direction'], Col.CYN)}  "
              f"-> ~{n} runs")
        print("\n   1-4) edit fc/fb/fv/kd_lat list (or 'off')   m) mode   d) dir")
        print("   R) RUN sweep                                 b) back")
        ch = ask("  choice: ").lower()
        edit = {"1": "fc", "2": "fb", "3": "fv", "4": "kd_lat"}
        if ch in edit:
            k = edit[ch]
            a = ask(f"  {k} comma-list (e.g. 20,25,30), 'off' to disable, blank=keep: ").strip()
            if a == "off":
                if k in sweep["enabled"]:
                    sweep["enabled"].remove(k)
            elif a:
                try:
                    vals = [float(x) for x in a.split(",") if x.strip()]
                    if vals:
                        sweep["params"][k] = vals
                        if k not in sweep["enabled"]:
                            sweep["enabled"].append(k)
                except ValueError:
                    pass
        elif ch == "m":
            sweep["mode"] = "grid" if sweep["mode"] == "descent" else "descent"
        elif ch == "d":
            sweep["direction"] = "cw" if sweep["direction"] == "ccw" else "ccw"
        elif ch == "r":
            run_autotest(node, geo, sweep, weights)
        elif ch == "b":
            return


def set_double(node, label, nodename, pname, allow_default=False):
    cur = node.get_params(nodename, [pname]).get(pname)
    curtxt = "default(NaN)" if (cur is None or (isinstance(cur, float) and math.isnan(cur))) else f"{cur}"
    extra = " (or 'd' for firmware default)" if allow_default else ""
    a = ask(f"  {label} (current {curtxt}){extra}: ").strip().lower()
    if a == "":
        return
    if a == "d" and allow_default:
        val = float("nan")
    else:
        try:
            val = float(a)
        except ValueError:
            print(c("  not a number", Col.RED))
            time.sleep(1.0)
            return
    ok = node.set_param(nodename, pname, val, ParameterType.PARAMETER_DOUBLE)
    print(c(f"  {'set' if ok else 'FAILED'}: {pname} = {val}", Col.GRN if ok else Col.RED))
    time.sleep(1.0)


def main():
    rclpy.init()
    node = PathCal()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    time.sleep(1.0)
    geo = {"mode": "area", "name": "TestArea", "inset": 0.0, "a": 3.0, "b": 2.0,
           "edges": "straight"}
    sweep = {
        "params": {"fc": [20.0, 25.0, 30.0], "fb": [35.0, 40.0, 45.0],
                   "fv": [120.0, 150.0, 180.0], "kd_lat": [1.0, 1.5, 2.0]},
        "enabled": ["fc", "fb"],
        "mode": "descent",
        "direction": "ccw",
    }
    # spd_target = cruise floor for the under-speed penalty. Mowing speed is
    # 0.5 m/s; 0.45 = "should nearly reach mowing cruise on the straights".
    weights = {"ct_rms": 1.0, "ct_peak": 0.5, "herr": 0.1, "wz": 2.0, "gz": 2.0,
               "uspd": 1.0, "spd_target": 0.45, "miss": 50.0}
    cw = ccw = None
    try:
        while True:
            cfg = read_cfg(node)
            header(node, cfg, geo)
            print("   1) Run outline CW          5) Set fc Coulomb")
            print("   2) Run outline CCW         6) Set fv Viscous")
            print("   3) Run BOTH + compare      7) Set fb Breakaway")
            print("   4) Show last summary       8) Set kd_lat")
            print("   c) COVERAGE path test (real F2C swaths + arcs)")
            print("   x) AUTO-TEST (sweep)       z) Rotation (yaw)  f) Fine-yaw")
            print("   p) Prep (RECORDING+clear)  9) Set derivative_filter_tau")
            print("   s) STOP (cancel motion)    g) Toggle gyro rate loop")
            print("   a) Outline source          o) Rect size")
            print("   v) Test cruise speed       r) Refresh    q) Quit")
            if node.hl_state not in HL_DRIVES:
                print(c("\n  NOTE: not in a driving mode -- press 'p' before running tests.", Col.YEL))
            ch = ask("\n  choice: ").lower()
            if ch == "1":
                cw = run_outline(node, "cw", geo)
                for ln in suggest(node, cw, None):
                    print(ln)
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "2":
                ccw = run_outline(node, "ccw", geo)
                for ln in suggest(node, None, ccw):
                    print(ln)
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "3":
                cw = run_outline(node, "cw", geo)
                time.sleep(1.0)
                ccw = run_outline(node, "ccw", geo)
                for ln in show_summary(node, cw, ccw):
                    print(ln)
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "4":
                if cw or ccw:
                    for ln in show_summary(node, cw, ccw):
                        print(ln)
                else:
                    print(c("  no runs yet", Col.YEL))
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "c":
                run_coverage(node, geo)
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "p":
                print("  setting RECORDING mode + clearing costmaps...")
                node.hl(3)
                time.sleep(1.0)
                node.clear_costmaps()
                print(c("  ready (RECORDING). Remember: blade OFF, area clear.", Col.GRN))
                time.sleep(1.5)
            elif ch == "s":
                node.cancel_follow()
                print(c("  follow cancelled", Col.GRN))
                time.sleep(0.8)
            elif ch == "5":
                set_double(node, "fc Coulomb [byte]", HB, "ff_coulomb_byte", allow_default=True)
            elif ch == "6":
                set_double(node, "fv Viscous [byte/(m/s)]", HB, "ff_viscous_byte_per_mps",
                           allow_default=True)
            elif ch == "7":
                set_double(node, "fb Breakaway [byte]", HB, "ff_breakaway_byte", allow_default=True)
            elif ch == "8":
                set_double(node, "FollowPath.kd_lat", CS, "FollowPath.kd_lat")
            elif ch == "9":
                set_double(node, "FollowPath.derivative_filter_tau [s]", CS,
                           "FollowPath.derivative_filter_tau")
            elif ch == "g":
                cur = node.get_params(HB, ["angular_rate_loop_enabled"]).get(
                    "angular_rate_loop_enabled")
                ok = node.set_param(HB, "angular_rate_loop_enabled", not cur,
                                    ParameterType.PARAMETER_BOOL)
                print(c(f"  gyro rate loop -> {'ON' if not cur else 'off'} "
                        f"({'ok' if ok else 'FAILED'})", Col.GRN if ok else Col.RED))
                time.sleep(1.0)
            elif ch in ("z", "f"):
                if ch == "f":
                    dflt, dreps = "1,-1,3,-3,5,-5", 3
                    prompt = "  small angles deg [1,-1,3,-3,5,-5]: "
                else:
                    dflt, dreps = "90,-90,45,-45", 1
                    prompt = "  target angles deg [90,-90,45,-45]: "
                a = ask(prompt).strip()
                try:
                    angles = [float(x) for x in a.split(",") if x.strip()] or \
                        [float(x) for x in dflt.split(",")]
                except ValueError:
                    angles = [float(x) for x in dflt.split(",")]
                rp = ask(f"  reps per angle [{dreps}]: ").strip()
                reps = dreps
                if rp:
                    try:
                        reps = max(1, int(float(rp)))
                    except ValueError:
                        pass
                run_rotation(node, angles, reps)
                ask(c("\n  ENTER...", Col.DIM))
            elif ch == "v":
                set_double(node, "FollowPath.speed_fast (test cruise, mowing=0.5)",
                           CS, "FollowPath.speed_fast")
            elif ch == "x":
                autotest_setup(node, geo, sweep, weights)
            elif ch == "a":
                m = ask("  source: [1] real area  [2] rectangle: ").strip()
                if m == "1":
                    nm = ask(f"  area name [{geo['name']}]: ").strip() or geo["name"]
                    geo["mode"] = "area"
                    geo["name"] = nm
                    iv = ask(f"  inset metres [{geo['inset']:.1f}] (0=raw boundary): ").strip()
                    if iv:
                        try:
                            geo["inset"] = max(0.0, float(iv))
                        except ValueError:
                            pass
                elif m == "2":
                    geo["mode"] = "rect"
                em = ask("  edges: [1] straight (measure line-following)  "
                         "[2] smac (curvy/feasible) [1]: ").strip()
                geo["edges"] = "smac" if em == "2" else "straight"
            elif ch == "o":
                for k, lbl, lo, hi in (("a", "length (forward) m", 1.0, 15.0),
                                       ("b", "width (lateral) m", 1.0, 15.0)):
                    av = ask(f"  outline {lbl} [{geo[k]:.1f}] (blank=keep): ").strip()
                    if av:
                        try:
                            geo[k] = max(lo, min(hi, float(av)))
                        except ValueError:
                            pass
            elif ch == "r":
                continue
            elif ch == "q":
                break
    finally:
        node.cancel_follow()
        rclpy.shutdown()
        print("bye")


if __name__ == "__main__":
    main()
