#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Drive-tuning node — automated maneuver runner + scalar-objective optimizer.

Successor to the v2-control-cascade ``path_calibrate.py`` TUI, rebuilt as a
persistent ROS 2 node on the dev stack (RotationShim+MPPI/RPP, ``plan_coverage``,
the runtime ``wheel_pid_*`` / ``deadband_pwm`` / ``hold_kp`` / ``wheel_pi``
firmware knobs) so the GUI drive-tuning panel can be a thin client.

It drives REALISTIC maneuvers (NOT hand-built straight lines, which give
artifact hunting):

  * ``outline``      — a closed polygon with sharp corners, tracked with the
                       transit controller (RotationShim+RPP). Stresses corner
                       pivots + straight tracking.
  * ``swath``        — a real ``plan_coverage`` swath segment around the current
                       pose, tracked with the coverage controller
                       (RotationShim+MPPI). Stresses swath-entry pivot + tracking.
  * ``transit_pose`` — a Smac (Hybrid-A*) path to a precise offset pose, then the
                       final-pose error is scored. Simulates a docking approach.
  * ``yaw_hunt``     — in-place ``Spin`` to a small target, high-rate gyro capture
                       with a post-spin settle window. Stresses fine yaw + the
                       deadband breakaway + the standstill position hold
                       (peak vs final vs back-creep).

For each run it scores the maneuver against the planned path / target and
publishes the metrics. With optimization enabled it runs a coordinate / finite-
difference descent over the drive params, applying each candidate live via the
``/hardware_bridge`` parameters and proposing the next combination to try.

Interfaces (so the GUI panel is a thin client):
  ~/command   (std_msgs/String, in)  — JSON: {"action": "run"|"optimize"|"stop",
                                        "maneuver": "<name>", ...overrides}
  ~/status    (std_msgs/String, out) — JSON: phase, last metrics, best-so-far,
                                        suggested next param combo, history.

PREREQUISITES (the node refuses to drive otherwise):
  * Robot OFF the dock, blade OFF, clear area — it drives autonomously.
  * Localization converged (sigma_xy small) and yaw anchored.
  * BT in a driving mode (the node requests RECORDING via HighLevelControl so
    Nav2 cmd_vel reaches the motors WITHOUT arming the blade).

Run in-container:
  docker exec -it mowgli-ros2 bash -lc \\
    'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \\
     python3 /ros2_ws/scripts/drive_tuning_node.py'
"""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point32, Polygon, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav2_msgs.srv import ClearEntireCostmap
from mowgli_interfaces.action import PlanCoverage
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Status
from mowgli_interfaces.srv import HighLevelControl

HB = "/hardware_bridge"  # drive-param node (firmware knobs)
CMD_RECORD_AREA = 3  # HighLevelControl: enter RECORDING (drives without blade)
CMD_RECORD_CANCEL = 6  # leave RECORDING without saving a polygon
HL_DRIVES = {2, 3, 4}  # AUTONOMOUS / RECORDING / MANUAL_MOWING accept cmd_vel

# Tunable drive parameters: name -> (ros_type, lo, hi, init_step). All live on
# /hardware_bridge and are pushed to the firmware via PKT_ID_SET_DRIVE_PID.
PARAM_SPECS = {
    "wheel_pid_deadband_pwm": ("double", 0.0, 120.0, 8.0),
    "wheel_pid_pwm_per_mps": ("double", 50.0, 800.0, 25.0),
    "wheel_pid_kp": ("double", 0.0, 200.0, 10.0),
    "wheel_pid_ki": ("double", 0.0, 20000.0, 1000.0),
    "wheel_pid_integral_limit": ("double", 20.0, 200.0, 20.0),
    "wheel_hold_kp": ("double", 0.0, 30.0, 2.0),
    "angular_rate_kp": ("double", 0.0, 2.0, 0.1),
    "angular_rate_ki": ("double", 0.0, 8.0, 0.5),
}

# wheel_pid_kd is deliberately ABSENT from PARAM_SPECS — the per-wheel velocity
# loop runs on quantized encoder ticks at 50 Hz, so a derivative term amplifies
# that quantization noise far more than it damps any real overshoot. The phases
# that look like they'd want D (breakaway "pop", pivot overshoot) are absorbed by
# the outer position loop (MPPI/RPP) and tuned by the angular-rate step instead.
# kd stays at 0 and is reachable only via the expert Drive Motor form; it would
# need a filtered derivative before it is safe to put under the optimizer. The
# firmware re-clamps integral_limit on receipt, so the host range above only
# bounds the search, not safety. See PROTOCOL_NOTES (surfaced to the operator).

# Guided multi-step tuning protocol — ordered by dependency: each step assumes
# the ones above it are already set (breakaway -> viscous -> PI trim -> hold ->
# angular loop). Each step optimizes a small param subset with the maneuver that
# best exercises it, and carries plain-language guidance for non-technical
# operators. Only params that exist in firmware today (no dither/pulse yet).
PROTOCOL = [
    {
        "id": "breakaway",
        "title": "1 · Breakaway (deadband)",
        "params": ["wheel_pid_deadband_pwm"],
        "maneuver": "transit_pose",
        "clearance": "~1.5 m clear straight ahead",
        "guidance": (
            "Foundation step. Place the robot with ~1.5 m of clear space straight "
            "ahead, blade OFF, off the dock. Finds the minimum PWM that reliably "
            "breaks the wheels away from rest. The robot drives a short distance "
            "forward to a precise pose."
        ),
    },
    {
        "id": "viscous",
        "title": "2 · Speed scale (viscous)",
        "params": ["wheel_pid_pwm_per_mps"],
        "maneuver": "transit_pose",
        "clearance": "~2.5 m clear straight ahead",
        "guidance": (
            "With breakaway set, scales PWM so the commanded speed matches the "
            "actual speed. Keep ~2.5 m clear straight ahead; the robot drives "
            "forward and stops on a precise pose."
        ),
    },
    {
        "id": "pid_trim",
        "title": "3 · Speed PI trim (optional)",
        "params": ["wheel_pid_kp", "wheel_pid_ki"],
        "maneuver": "transit_pose",
        "clearance": "~2.5 m clear straight ahead",
        "optional": True,
        "guidance": (
            "Only if closed-loop PI is enabled (wheel_pi_enabled). Trims residual "
            "speed error on top of breakaway + viscous. Skip when running "
            "open-loop feedforward."
        ),
    },
    {
        "id": "windup",
        "title": "3b · Precise-stop (windup bound)",
        "params": ["wheel_pid_integral_limit"],
        "maneuver": "transit_pose",
        "clearance": "~2.5 m clear straight ahead",
        "optional": True,
        "guidance": (
            "Run AFTER the PI trim, and only if closed-loop PI is enabled "
            "(wheel_pi_enabled) — skip when running open-loop feedforward. Bounds "
            "how far the speed integrator may wind up, so the robot stops "
            "precisely instead of lurching past the target after a slow approach "
            "or a stall. Scored by final-pose error, so this is the step that "
            "protects docking and segment-end stop accuracy. Keep ~2.5 m clear "
            "straight ahead; the robot drives forward and stops on a precise pose."
        ),
    },
    {
        "id": "hold",
        "title": "4 · Standstill hold",
        "params": ["wheel_hold_kp"],
        "maneuver": "yaw_hunt",
        "clearance": "~1 m clear radius",
        "guidance": (
            "Needs ~1 m clear all around for an in-place pivot. Tunes how firmly "
            "the robot holds position after a turn, killing back-creep. The robot "
            "pivots then settles."
        ),
    },
    {
        "id": "angular",
        "title": "5 · Turn rate (angular loop)",
        "params": ["angular_rate_kp", "angular_rate_ki"],
        "maneuver": "yaw_hunt",
        "clearance": "~1 m clear radius",
        "guidance": (
            "Same ~1 m clear radius. Tunes how accurately the robot reaches a "
            "commanded heading with small in-place pivots."
        ),
    },
]

# Surfaced to the operator alongside the protocol (sent on get_protocol). Explains
# why the drive derivative gain is not part of the guided flow.
PROTOCOL_NOTES = (
    "wheel_pid_kd (drive derivative gain) is intentionally NOT auto-tuned — it is "
    "left at 0 and exposed only in the expert Drive Motor settings. On this "
    "quantized 50 Hz velocity loop a derivative term amplifies encoder noise more "
    "than it helps; onset/pivot overshoot is handled by the position loop and the "
    "turn-rate step instead."
)


def yq(yaw):
    """(z, w) quaternion for a yaw about +Z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _rms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return 0.0
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def _zero_cross_rate(series, dt_total):
    """Sign-change count per second — a hunting / oscillation proxy."""
    if dt_total <= 0.0 or len(series) < 2:
        return 0.0
    crossings = 0
    for a, b in zip(series, series[1:]):
        if (a > 0.0 and b < 0.0) or (a < 0.0 and b > 0.0):
            crossings += 1
    return crossings / dt_total


def _cross_track(px, py, path_xy):
    """Min distance from (px,py) to the planned polyline (segment-wise)."""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(path_xy, path_xy[1:]):
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = min(best, d)
    return best


class DriveTuning(Node):
    def __init__(self):
        super().__init__("drive_tuning_node")
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ---- declared parameters (run config; overridable per command) -----
        self.declare_parameter("maneuver", "yaw_hunt")
        self.declare_parameter("optimize_max_iters", 24)
        self.declare_parameter("yaw_hunt_target_rad", 0.5)
        self.declare_parameter("outline_size_m", 2.0)
        self.declare_parameter("transit_dist_m", 1.5)
        self.declare_parameter("swath_box_m", 4.0)
        # Objective weights (per-metric). Scaled so each term is ~O(1).
        self.declare_parameter("w_cross_track", 4.0)
        self.declare_parameter("w_heading", 1.0)
        self.declare_parameter("w_hunting", 0.5)
        self.declare_parameter("w_backcreep", 2.0)
        self.declare_parameter("w_pose_err", 5.0)

        # ---- state ----------------------------------------------------------
        self.x = self.y = self.yaw = self.sx = None
        self._vx = self._wz = self._gz = 0.0
        self._wheel_v = 0.0
        self.hl_state = None
        self.emergency = False
        self.is_charging = None
        self.recording = False
        self.samples = []
        self._gz_buf = []
        self._capture_gyro = False
        self._fp_gh = None
        self._busy = False
        self._stop = False
        self.armed = False  # live subscriptions active (set by start_session)
        self._lock = threading.Lock()
        self._status = {"phase": "idle"}
        self._set = {}
        self._get = {}

        # ---- I/O ------------------------------------------------------------
        # Always-on, lightweight: BT/safety state + the command/status channel.
        # The heavy pose/IMU/cmd_vel subscriptions are created lazily by
        # start_session() so the node can stay launched (default on) at
        # negligible idle cost and only listen while a session is armed.
        self._rel = rel
        self.create_subscription(HighLevelStatus, "/behavior_tree_node/high_level_status",
                                 self._hl, rel)
        self.create_subscription(Emergency, HB + "/emergency", self._emg, rel)
        self.create_subscription(Status, HB + "/status", self._stat, rel)
        self.create_subscription(String, "~/command", self._on_command, rel)
        self.pub_status = self.create_publisher(String, "~/status", rel)
        self.create_timer(1.0, self._publish_status)
        self._session_subs = []  # odom/wheel/cmd_vel/imu — live only while armed

        self.cp = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.fp = ActionClient(self, FollowPath, "/follow_path")
        self.sp = ActionClient(self, Spin, "/spin")
        self.cov = ActionClient(self, PlanCoverage, "/plan_coverage")
        self.hlc = self.create_client(HighLevelControl,
                                      "/behavior_tree_node/high_level_control")
        self.clr_g = self.create_client(ClearEntireCostmap,
                                        "/global_costmap/clear_entirely_global_costmap")
        self.clr_l = self.create_client(ClearEntireCostmap,
                                        "/local_costmap/clear_entirely_local_costmap")
        self.get_logger().info("drive_tuning_node ready — send JSON to ~/command")

    # ---- callbacks ----------------------------------------------------------
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
        self._wheel_v = m.twist.twist.linear.x

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

    def _stat(self, m):
        self.is_charging = m.is_charging

    # ---- helpers ------------------------------------------------------------
    def _await(self, fut, timeout):
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _set_cli(self, node):
        self._set.setdefault(node, self.create_client(SetParameters,
                                                      node + "/set_parameters"))
        return self._set[node]

    def _get_cli(self, node):
        self._get.setdefault(node, self.create_client(GetParameters,
                                                      node + "/get_parameters"))
        return self._get[node]

    def set_param(self, node, name, value, ptype):
        cli = self._set_cli(node)
        if not cli.wait_for_service(timeout_sec=3.0):
            return False
        pv = ParameterValue(type=ptype)
        if ptype == ParameterType.PARAMETER_DOUBLE:
            pv.double_value = float(value)
        elif ptype == ParameterType.PARAMETER_BOOL:
            pv.bool_value = bool(value)
        res = self._await(cli.call_async(
            SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])), 3.0)
        return bool(res and res.results and res.results[0].successful)

    def apply_params(self, combo):
        ok = True
        for name, val in combo.items():
            spec = PARAM_SPECS.get(name)
            if spec is None:
                continue
            pt = (ParameterType.PARAMETER_DOUBLE if spec[0] == "double"
                  else ParameterType.PARAMETER_BOOL)
            ok = self.set_param(HB, name, val, pt) and ok
        time.sleep(0.3)  # let the bridge push PKT_ID_SET_DRIVE_PID to the STM32
        return ok

    def get_combo(self):
        cli = self._get_cli(HB)
        names = list(PARAM_SPECS.keys())
        out = {}
        if cli.wait_for_service(timeout_sec=3.0):
            res = self._await(cli.call_async(GetParameters.Request(names=names)), 3.0)
            if res:
                for n, pv in zip(names, res.values):
                    if pv.type == ParameterType.PARAMETER_DOUBLE:
                        out[n] = pv.double_value
        for n, spec in PARAM_SPECS.items():
            out.setdefault(n, spec[1])
        return out

    def hl(self, cmd):
        if not self.hlc.wait_for_service(timeout_sec=5.0):
            return False
        return self._await(self.hlc.call_async(
            HighLevelControl.Request(command=cmd)), 5.0) is not None

    def clear_costmaps(self):
        for cl in (self.clr_g, self.clr_l):
            if cl.wait_for_service(timeout_sec=3.0):
                self._await(cl.call_async(ClearEntireCostmap.Request()), 3.0)

    def preflight(self):
        if self.x is None or self.sx is None:
            return "no localization"
        if self.sx > 0.30:
            return f"localization degraded (sigma_xy={self.sx:.2f} m)"
        if self.emergency:
            return "emergency latched"
        if self.is_charging:
            return "on dock (undock first)"
        return None

    def ensure_driving(self):
        if self.hl_state in HL_DRIVES:
            return True
        self.hl(CMD_RECORD_AREA)
        t0 = time.time()
        while time.time() - t0 < 5.0 and self.hl_state not in HL_DRIVES:
            time.sleep(0.1)
        return self.hl_state in HL_DRIVES

    # ---- session lifecycle (lazy heavy subscriptions) -----------------------
    def start_session(self):
        """Arm: create the live pose/IMU/cmd_vel subscriptions. Idempotent."""
        if self.armed:
            return
        self._session_subs = [
            self.create_subscription(Odometry, "/odometry/filtered_map", self._odom, 10),
            self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, self._rel),
            self.create_subscription(TwistStamped, "/cmd_vel_nav", self._cmd, 10),
            self.create_subscription(Imu, "/imu/data", self._imu, qos_profile_sensor_data),
        ]
        self.armed = True
        self._set_phase("session active")
        self.get_logger().info("tuning session armed — live subscriptions active")

    def stop_session(self):
        """Disarm: cancel any maneuver and release the live subscriptions."""
        if not self.armed:
            return
        self.cancel()  # abort a running maneuver/optimize
        for sub in self._session_subs:
            self.destroy_subscription(sub)
        self._session_subs = []
        # Drop cached pose so a stale value can't pass preflight after re-arm.
        self.x = self.y = self.yaw = self.sx = None
        self.armed = False
        self.hl(CMD_RECORD_CANCEL)
        self._set_phase("idle")
        self.get_logger().info("tuning session stopped — live subscriptions released")

    # ---- path builders ------------------------------------------------------
    def _densify(self, pts, yaws, step=0.05):
        p = Path()
        p.header.frame_id = "map"
        for (ax, ay), (bx, by), yaw in zip(pts, pts[1:], yaws):
            d = math.hypot(bx - ax, by - ay)
            n = max(1, int(d / step))
            qz, qw = yq(yaw)
            for k in range(n + 1):
                t = k / n
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = ax + (bx - ax) * t
                ps.pose.position.y = ay + (by - ay) * t
                ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
                p.poses.append(ps)
        return p

    def outline_path(self, size):
        """A closed diamond around the current pose — four sharp corners."""
        cx, cy, h = self.x, self.y, size / 2.0
        corners = [(cx + h, cy), (cx, cy + h), (cx - h, cy), (cx, cy - h), (cx + h, cy)]
        yaws = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(corners, corners[1:])]
        return self._densify(corners, yaws)

    def plan_smac(self, gx, gy, gyaw):
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
        return res.result.path if res and res.result.path.poses else None

    def coverage_swath(self, box):
        """plan_coverage over a box around the pose; return the first swath
        segment (SEGMENT_SWATH) as a drivable Path."""
        if not self.cov.wait_for_server(timeout_sec=8.0):
            return None
        cx, cy, h = self.x, self.y, box / 2.0
        poly = Polygon(points=[Point32(x=cx - h, y=cy - h, z=0.0),
                               Point32(x=cx + h, y=cy - h, z=0.0),
                               Point32(x=cx + h, y=cy + h, z=0.0),
                               Point32(x=cx - h, y=cy + h, z=0.0)])
        g = PlanCoverage.Goal(outer_boundary=poly, obstacles=[], mow_angle_deg=0.0)
        gh = self._await(self.cov.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            return None
        res = self._await(gh.get_result_async(), 30.0)
        if not res or not res.result.success:
            return None
        for seg, kind in zip(res.result.segments, res.result.segment_types):
            if kind == PlanCoverage.Result.SEGMENT_SWATH and seg.poses:
                return seg
        return res.result.segments[0] if res.result.segments else None

    # ---- maneuver execution -------------------------------------------------
    def follow(self, path, controller_id, goal_checker_id, timeout=180.0):
        if not self.fp.wait_for_server(timeout_sec=6.0):
            return "NO_SRV"
        self.samples = []
        self.recording = True
        g = FollowPath.Goal(path=path, controller_id=controller_id,
                            goal_checker_id=goal_checker_id)
        gh = self._await(self.fp.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            self.recording = False
            return "REJECTED"
        self._fp_gh = gh
        res = self._await(gh.get_result_async(), timeout)
        self.recording = False
        self._fp_gh = None
        if not res:
            return "TIMEOUT"
        return {4: "OK", 5: "CANCEL", 6: "ABORT"}.get(res.status, str(res.status))

    def spin(self, target_rad, timeout=40.0):
        if not self.sp.wait_for_server(timeout_sec=6.0):
            return "NO_SRV"
        self.samples = []
        self._gz_buf = []
        self.recording = True
        self._capture_gyro = True
        g = Spin.Goal(target_yaw=float(target_rad),
                      time_allowance=Duration(sec=int(timeout)))
        gh = self._await(self.sp.send_goal_async(g), 8.0)
        if not gh or not gh.accepted:
            self.recording = self._capture_gyro = False
            return "REJECTED"
        self._fp_gh = gh
        res = self._await(gh.get_result_async(), timeout + 10.0)
        time.sleep(1.0)  # settle window: capture coast-back creep at zero command
        self.recording = self._capture_gyro = False
        self._fp_gh = None
        if not res:
            return "TIMEOUT"
        return {4: "OK", 5: "CANCEL", 6: "ABORT"}.get(res.status, str(res.status))

    def cancel(self):
        self._stop = True
        if self._fp_gh is not None:
            try:
                self._await(self._fp_gh.cancel_goal_async(), 3.0)
            except Exception:
                pass

    # ---- metrics ------------------------------------------------------------
    def metrics_path(self, path):
        """Cross-track / heading RMS + hunting against the planned polyline."""
        path_xy = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(path_xy) < 2 or not self.samples:
            return {}
        ct, herr, wz, gz = [], [], [], []
        for (_, x, y, yaw, vx, w, g, _) in self.samples:
            ct.append(_cross_track(x, y, path_xy))
            # nearest planned heading
            pq = path.poses[min(range(len(path.poses)),
                                key=lambda i: (path.poses[i].pose.position.x - x) ** 2
                                + (path.poses[i].pose.position.y - y) ** 2)].pose.orientation
            pyaw = math.atan2(2 * pq.w * pq.z, 1 - 2 * pq.z * pq.z)
            herr.append(abs(wrap(yaw - pyaw)))
            wz.append(w)
            gz.append(g)
        dt = self.samples[-1][0] - self.samples[0][0]
        return {
            "ct_rms": _rms(ct), "ct_peak": max(ct),
            "herr_rms": _rms(herr), "herr_peak": max(herr),
            "wz_zc": _zero_cross_rate(wz, dt), "gz_zc": _zero_cross_rate(gz, dt),
            "samples": len(self.samples),
        }

    def metrics_yaw(self, target):
        """Peak / final / back-creep from high-rate gyro integration."""
        if len(self._gz_buf) < 2:
            return {}
        ang, integ = [], 0.0
        for (t0, g0), (t1, _) in zip(self._gz_buf, self._gz_buf[1:]):
            integ += g0 * (t1 - t0)
            ang.append(integ)
        peak = max(ang, key=abs)
        final = ang[-1]
        # back-creep: reversal from the peak after the command ended
        back = peak - final if abs(peak) > abs(final) else 0.0
        gz = [g for _, g in self._gz_buf]
        dt = self._gz_buf[-1][0] - self._gz_buf[0][0]
        return {
            "yaw_peak": peak, "yaw_final": final, "yaw_target": target,
            "undershoot": target - abs(final) if target else 0.0,
            "backcreep": abs(back), "gz_zc": _zero_cross_rate(gz, dt),
        }

    def metrics_pose(self, gx, gy, gyaw):
        """Final-pose error vs a precise target (docking sim)."""
        return {
            "pose_xy_err": math.hypot(self.x - gx, self.y - gy),
            "pose_yaw_err": abs(wrap(self.yaw - gyaw)),
        }

    def score(self, maneuver, m):
        """Scalar objective (lower = better)."""
        g = self.get_parameter
        w_ct = g("w_cross_track").value
        w_h = g("w_heading").value
        w_hunt = g("w_hunting").value
        w_bc = g("w_backcreep").value
        w_pe = g("w_pose_err").value
        if maneuver == "yaw_hunt":
            return (w_h * m.get("undershoot", 0.0) + w_bc * m.get("backcreep", 0.0)
                    + w_hunt * 0.1 * m.get("gz_zc", 0.0))
        if maneuver == "transit_pose":
            return (w_pe * m.get("pose_xy_err", 0.0) + w_h * m.get("pose_yaw_err", 0.0))
        # outline / swath: path tracking
        return (w_ct * m.get("ct_rms", 0.0) + w_h * m.get("herr_rms", 0.0)
                + w_hunt * 0.1 * m.get("wz_zc", 0.0))

    # ---- one maneuver run ---------------------------------------------------
    def run_maneuver(self, maneuver):
        pre = self.preflight()
        if pre:
            return {"error": pre}
        if not self.ensure_driving():
            return {"error": "could not enter a driving BT mode"}
        self.clear_costmaps()
        self._stop = False
        try:
            if maneuver == "yaw_hunt":
                tgt = float(self.get_parameter("yaw_hunt_target_rad").value)
                st = self.spin(tgt)
                m = self.metrics_yaw(tgt)
            elif maneuver == "transit_pose":
                d = float(self.get_parameter("transit_dist_m").value)
                gx, gy = self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw)
                gyaw = self.yaw
                path = self.plan_smac(gx, gy, gyaw)
                if path is None:
                    return {"error": "Smac plan failed"}
                st = self.follow(path, "FollowPath", "stopped_goal_checker")
                m = self.metrics_pose(gx, gy, gyaw)
            elif maneuver == "swath":
                path = self.coverage_swath(float(self.get_parameter("swath_box_m").value))
                if path is None:
                    return {"error": "plan_coverage produced no swath"}
                st = self.follow(path, "FollowCoveragePath", "coverage_goal_checker")
                m = self.metrics_path(path)
            else:  # outline
                path = self.outline_path(float(self.get_parameter("outline_size_m").value))
                st = self.follow(path, "FollowPath", "stopped_goal_checker")
                m = self.metrics_path(path)
        except Exception as exc:  # noqa: BLE001 — surface any driver error to the GUI
            return {"error": f"{type(exc).__name__}: {exc}"}
        m["follow_status"] = st
        m["score"] = self.score(maneuver, m)
        return m

    def _return_to_start(self, start):
        """Drive back to the pose captured at the start of optimize() so the
        descent's footprint stays bounded (~one maneuver length) no matter how
        many iterations run, instead of marching cumulatively forward. Returns
        False if it cannot get back (so the descent aborts rather than walking
        off the cleared area). A clean user stop counts as success."""
        if self._stop:
            return True
        if self.x is None:
            return False
        sx, sy, syaw = start
        self._set_phase("returning to start")
        # 1) Translate back along the same corridor if we moved. RotationShim
        #    pivots ~180 deg in place, RPP drives the straight path back.
        dist = math.hypot(sx - self.x, sy - self.y)
        if dist > 0.08:
            heading_back = math.atan2(sy - self.y, sx - self.x)
            path = self._densify([(self.x, self.y), (sx, sy)], [heading_back])
            st = self.follow(path, "FollowPath", "stopped_goal_checker", timeout=90.0)
            if st != "OK" and not self._stop:
                # Retry via Smac in case the hand-built straight path was rejected.
                p2 = self.plan_smac(sx, sy, syaw)
                st = (self.follow(p2, "FollowPath", "stopped_goal_checker", timeout=90.0)
                      if p2 is not None else st)
            if st != "OK" and not self._stop:
                return False
        # 2) Restore the start heading so the next forward run is comparable.
        if self.x is not None and not self._stop:
            dyaw = wrap(syaw - self.yaw)
            if abs(dyaw) > 0.10:
                self.spin(dyaw)
        return True

    # ---- coordinate / finite-difference descent -----------------------------
    def optimize(self, maneuver, param_names=None):
        max_iters = int(self.get_parameter("optimize_max_iters").value)
        # param_names lets a protocol step optimize only its own subset; the rest
        # of the combo is read + reapplied unchanged. Default = sweep everything.
        names = list(param_names) if param_names else list(PARAM_SPECS.keys())
        combo = self.get_combo()
        steps = {n: PARAM_SPECS[n][3] for n in names}
        self.apply_params(combo)
        # Anchor the footprint: every scored run returns here, so the descent
        # needs only ~one maneuver of clearance regardless of iteration count.
        if self.x is None:
            return {"error": "no localization — cannot anchor return-to-start"}
        start = (self.x, self.y, self.yaw)
        base = self.run_maneuver(maneuver)
        if "error" in base:
            return base
        if not self._return_to_start(start):
            return {"error": "could not return to start pose — aborting to keep "
                             "the working area bounded"}
        best, best_score = dict(combo), base["score"]
        history = [{"combo": dict(combo), "metrics": base, "score": best_score}]
        idx = 0
        for it in range(max_iters):
            if self._stop:
                break
            name = names[idx % len(names)]
            idx += 1
            lo, hi = PARAM_SPECS[name][1], PARAM_SPECS[name][2]
            improved = False
            for direction in (+1.0, -1.0):
                if self._stop:
                    break
                cand = dict(best)
                cand[name] = min(hi, max(lo, best[name] + direction * steps[name]))
                if abs(cand[name] - best[name]) < 1e-9:
                    continue
                self._set_phase(f"opt it={it} try {name}{'+' if direction > 0 else '-'}",
                                next_combo=cand, best=best, best_score=best_score,
                                history=history)
                self.apply_params(cand)
                res = self.run_maneuver(maneuver)
                if "error" in res:
                    return res
                history.append({"combo": dict(cand), "metrics": res, "score": res["score"]})
                if not self._return_to_start(start):
                    return {"error": "could not return to start pose — aborting to "
                                     "keep the working area bounded"}
                if res["score"] < best_score - 1e-6:
                    best, best_score, improved = dict(cand), res["score"], True
                    break
            if not improved:
                steps[name] *= 0.5  # shrink this coordinate's step on no gain
            if all(s < PARAM_SPECS[n][3] * 0.2 for n, s in steps.items()):
                break  # converged: every step shrank below 20% of its initial
        self.apply_params(best)  # leave the robot on the best combo
        return {"best": best, "best_score": best_score, "iters": len(history),
                "history": history}

    # ---- command + status ---------------------------------------------------
    def _set_phase(self, phase, **extra):
        with self._lock:
            self._status = {"phase": phase, "t": time.time(), **extra}

    def _publish_status(self):
        with self._lock:
            snap = dict(self._status)
        snap["busy"] = self._busy
        snap["armed"] = self.armed
        snap["hl_state"] = self.hl_state
        snap["sigma_xy"] = self.sx
        try:
            self.pub_status.publish(String(data=json.dumps(snap, default=str)))
        except (TypeError, ValueError):
            self.pub_status.publish(String(data=json.dumps({"phase": snap.get("phase")})))

    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f"ignoring malformed ~/command JSON: {msg.data!r}")
            return
        action = cmd.get("action")
        if action == "stop":
            self.cancel()
            return
        if action == "start_session":
            self.start_session()
            return
        if action == "stop_session":
            self.stop_session()
            return
        if action == "get_protocol":
            # One-off: hand the ordered step list + guidance to the GUI stepper.
            self.pub_status.publish(String(data=json.dumps(
                {"protocol": PROTOCOL, "notes": PROTOCOL_NOTES})))
            return
        if self._busy:
            self.get_logger().warn("busy — ignoring command")
            return
        for k, v in cmd.items():  # allow per-command parameter overrides
            if k not in ("action", "maneuver", "step") and self.has_parameter(k):
                try:
                    self.set_parameters([rclpy.parameter.Parameter(k, value=v)])
                except Exception:  # noqa: BLE001
                    pass
        # Step-scoped actions resolve their maneuver + param subset from PROTOCOL.
        params = None
        if action in ("run_step", "optimize_step"):
            step = self._find_step(cmd.get("step"))
            if step is None:
                self.get_logger().warn(f"unknown protocol step: {cmd.get('step')!r}")
                return
            maneuver = step["maneuver"]
            params = step["params"]
            action = "optimize" if action == "optimize_step" else "run"
        else:
            maneuver = cmd.get("maneuver", self.get_parameter("maneuver").value)
        if not self.armed:
            self.get_logger().warn("not armed — send start_session before running maneuvers")
            self._set_phase("not armed — press Start tuning")
            return
        threading.Thread(target=self._campaign, args=(action, maneuver, params),
                         daemon=True).start()

    def _find_step(self, step_id):
        return next((s for s in PROTOCOL if s["id"] == step_id), None)

    def _campaign(self, action, maneuver, params=None):
        self._busy = True
        self._stop = False
        try:
            if action == "optimize":
                self._set_phase(f"optimizing {maneuver}", params=params)
                result = self.optimize(maneuver, params)
            else:
                self._set_phase(f"running {maneuver}")
                result = self.run_maneuver(maneuver)
            self._set_phase(f"done {maneuver}", result=result)
            self.get_logger().info(f"{action} {maneuver}: {json.dumps(result, default=str)}")
        finally:
            self._busy = False
            # Leave RECORDING so the BT returns to idle cleanly.
            self.hl(CMD_RECORD_CANCEL)


def main():
    rclpy.init()
    node = DriveTuning()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
