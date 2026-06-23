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
import os
import re
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
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Status, WheelTick
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
# Params surfaced in the GUI "tunable parameters" panel: the optimizer doubles
# plus the wheel-PI bool. The staircase / optimize set these live; the panel
# shows the current value of each and flags those that differ from the saved
# config (overridden by a test) or are absent from it (firmware default).
PANEL_PARAM_NAMES = list(PARAM_SPECS.keys()) + ["wheel_pi_enabled"]

# hardware_bridge_node.cpp declare_parameter defaults — the value each tunable
# takes "from scratch" when it is NOT set in mowgli_robot.yaml (most of these
# are absent by default). The panel's Default column is the YAML value if the
# key is persisted, else this. Keep in sync with the bridge.
FIRMWARE_DEFAULTS = {
    "wheel_pid_kp": 30.0,
    "wheel_pid_ki": 5000.0,
    "wheel_pid_integral_limit": 100.0,
    "wheel_pid_pwm_per_mps": 300.0,
    "wheel_pid_deadband_pwm": 0.0,
    "wheel_hold_kp": 4.0,
    "angular_rate_kp": 0.4,
    "angular_rate_ki": 2.0,
    "wheel_pi_enabled": True,
}

PROTOCOL = [
    {
        "id": "breakaway",
        "title": "1 · Breakaway (deadband)",
        "params": ["wheel_pid_deadband_pwm"],
        "maneuver": "breakaway_staircase",
        "clearance": "~1.5 m clear straight ahead",
        "pi_run": "off",  # open-loop plant ID — the integrator would mask breakaway
        "guidance": (
            "Foundation step. Place the robot with ~1.5 m of clear space straight "
            "ahead, blade OFF, off the dock. Press Run: the robot ramps PWM up "
            "from rest and watches each drive wheel, finding the minimum PWM that "
            "breaks each wheel away. The deadband is set to the higher of the two "
            "(+ margin) so both wheels reliably start. This step has no Optimize — "
            "Run measures and sets it directly."
        ),
    },
    {
        "id": "viscous",
        "title": "2 · Speed scale (viscous)",
        "params": ["wheel_pid_pwm_per_mps"],
        "maneuver": "viscous_fit",
        "clearance": "~2.5 m clear straight ahead",
        "pi_run": "off",  # open-loop: the integrator hides a wrong speed scale
        "guidance": (
            "With breakaway set, scales PWM so commanded speed matches actual "
            "speed. Press Run: the robot drives forward at four speeds spanning "
            "your configured mowing/transit range and fits PWM-vs-speed, so the "
            "scale is calibrated where the robot actually drives (not extrapolated "
            "from one low speed). The fit's R² also flags if the response isn't "
            "linear over that range. Keep ~2.5 m clear ahead. Run, not Optimize."
        ),
    },
    {
        "id": "pid_trim",
        "title": "3 · Speed PI trim (optional)",
        "params": ["wheel_pid_kp", "wheel_pid_ki"],
        "maneuver": "transit_pose",
        "clearance": "~2.5 m clear straight ahead",
        "optional": True,
        "requires_pi": True,
        "pi_run": "on",  # closed-loop trim on top of the identified feedforward
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
        "requires_pi": True,
        "pi_run": "on",  # the integrator must be active to bound its windup
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


def _linfit(xs, ys):
    """Ordinary least-squares fit y = slope*x + intercept. Returns
    (slope, intercept, r2) or None if degenerate. Used by the viscous step to
    fit PWM vs achieved speed across several operating points — the slope is the
    feedforward gain (pwm_per_mps) and r2 reports how linear the plant actually
    is over the tested range (low r2 → the affine model is only approximate)."""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    intercept = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 1.0
    return slope, intercept, r2


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
        # Open-loop straight-drive (breakaway/viscous ID): constant speed of the
        # direct cmd_vel drive, and the weight on achieved-speed error.
        self.declare_parameter("open_loop_speed_mps", 0.20)
        self.declare_parameter("w_speed", 3.0)

        # ---- state ----------------------------------------------------------
        self.x = self.y = self.yaw = self.sx = None
        self._vx = self._wz = self._gz = 0.0
        self._wheel_v = 0.0
        self._ticks_rl = 0  # per-wheel cumulative encoder counts (breakaway staircase)
        self._ticks_rr = 0
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
        self.wheel_pi_enabled = None  # cached /hardware_bridge bool, surfaced to GUI
        self._live_params = {}   # name -> live /hardware_bridge value (GUI panel)
        self._saved_params = {}  # name -> mowgli_robot.yaml value (absent = firmware default)
        self._pi_poll_tick = 0
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
        # Direct teleop twist for open-loop straight drives (breakaway/viscous).
        # twist_mux 'teleop' input (priority 20 > navigation 10) reaches the
        # motors in RECORDING WITHOUT engaging RPP's outer heading loop — the
        # ID steps must bypass RPP (see drive_straight_open_loop).
        self.pub_teleop = self.create_publisher(TwistStamped, "/cmd_vel_teleop", rel)
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

    def _on_ticks(self, m):
        # Rear wheels are the drive wheels (2-wheel diff-drive maps to RL/RR).
        self._ticks_rl = int(m.wheel_ticks_rl)
        self._ticks_rr = int(m.wheel_ticks_rr)

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
        # Return ONLY what was actually read. Do NOT fabricate values for params
        # that failed to read — apply_params() writes the combo straight to the
        # firmware, so substituting a lower bound here silently zeroed real gains
        # (kp/ki/hold_kp/…) whenever a read hiccuped (e.g. the get_parameters
        # service not ready right after a node/bridge restart).
        return out

    def _refresh_live_params(self):
        """Async-cache the live value of every tunable on /hardware_bridge for
        the GUI panel (current value of each param) AND the wheel_pi_enabled
        gate. Fire-and-forget on purpose: this runs from the status-timer
        callback, where a blocking service wait deadlocks the executor — let the
        done-callback land the values asynchronously."""
        cli = self._get_cli(HB)
        if not cli.service_is_ready():
            return
        cli.call_async(
            GetParameters.Request(names=PANEL_PARAM_NAMES)
        ).add_done_callback(self._on_live_params_response)

    def _on_live_params_response(self, fut):
        # Ignore responses that land during a campaign — the tuner toggles params
        # per step then, so these would show transient mid-test values.
        if self._busy:
            return
        try:
            res = fut.result()
        except Exception:  # noqa: BLE001 — transient service error; retry next poll
            return
        if not res or not res.values:
            return
        out = {}
        for name, pv in zip(PANEL_PARAM_NAMES, res.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_BOOL:
                out[name] = pv.bool_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out[name] = pv.integer_value
        if out:
            self._live_params = out
            if "wheel_pi_enabled" in out:
                self.wheel_pi_enabled = out["wheel_pi_enabled"]

    def _read_saved_params(self):
        """Parse the persisted value of each tunable from mowgli_robot.yaml so
        the GUI can flag live values that differ from what survives a restart,
        and which keys are absent entirely (= firmware default). Cheap local
        read; cached in _saved_params."""
        saved = {}
        try:
            with open(self.ROBOT_YAML, "r") as f:
                text = f.read()
        except OSError:
            self._saved_params = saved
            return
        want = set(PANEL_PARAM_NAMES)
        key_re = re.compile(r"^\s*([A-Za-z0-9_]+)\s*:\s*([^#\n]+?)\s*(?:#.*)?$")
        for line in text.splitlines():
            m = key_re.match(line)
            if not m or m.group(1) not in want:
                continue
            raw = m.group(2).strip()
            if raw in ("true", "false"):
                saved[m.group(1)] = (raw == "true")
            else:
                try:
                    saved[m.group(1)] = float(raw)
                except ValueError:
                    pass
        self._saved_params = saved

    def _build_params_panel(self):
        """Per-param structure the GUI panel renders:
          {name: {live, default, persisted, lo, hi, step | is_bool}}
        - live      = current /hardware_bridge value (used now, lost on restart)
        - default   = the from-scratch boot value: the mowgli_robot.yaml value if
                      the key is persisted, else the firmware/bridge default
        - persisted = whether the key is in the YAML (else it's a firmware default)
        - lo/hi/step (doubles) or is_bool — bounds for the inline Edit control."""
        panel = {}
        for name in PANEL_PARAM_NAMES:
            live = self._live_params.get(name)
            if live is None:
                continue
            persisted = name in self._saved_params
            default = self._saved_params.get(name) if persisted else FIRMWARE_DEFAULTS.get(name)
            cell = {"live": live, "default": default, "persisted": persisted}
            if name in PARAM_SPECS:
                _, lo, hi, step = PARAM_SPECS[name]
                cell.update(lo=lo, hi=hi, step=step)
            else:
                cell["is_bool"] = True
            panel[name] = cell
        return panel

    def _read_wheel_pi_blocking(self):
        """Synchronous read of the operating wheel_pi_enabled — safe ONLY from a
        worker thread (e.g. _campaign), never from an executor callback."""
        cli = self._get_cli(HB)
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        res = self._await(cli.call_async(
            GetParameters.Request(names=["wheel_pi_enabled"])), 2.0)
        if res and res.values and res.values[0].type == ParameterType.PARAMETER_BOOL:
            return res.values[0].bool_value
        return None

    def _get_param_values(self, names):
        """Blocking mixed-type read of params from /hardware_bridge. Worker-thread
        ONLY (uses wait_for_service). Returns {name: value} for those resolved."""
        cli = self._get_cli(HB)
        out = {}
        if not cli.wait_for_service(timeout_sec=2.0):
            return out
        res = self._await(cli.call_async(GetParameters.Request(names=list(names))), 2.0)
        if not res:
            return out
        for n, pv in zip(names, res.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                out[n] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_BOOL:
                out[n] = pv.bool_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out[n] = pv.integer_value
        return out

    # ---- persistence (write tuned params to mowgli_robot.yaml) ---------------
    # Path that mowgli.launch.py reads at boot (mowgli.launch.py:77). In the
    # mowgli-ros2 container this is the host-mounted docker/config/mowgli file,
    # so a write here survives a container restart AND is the same file the GUI
    # Drive Motor settings form edits.
    ROBOT_YAML = "/ros2_ws/config/mowgli_robot.yaml"

    def persist_params(self):
        """Write the live /hardware_bridge drive params to mowgli_robot.yaml so
        they survive a restart (the optimizer / staircase otherwise only sets
        them live and the firmware reverts to the saved values on reconnect).
        Worker-thread only — get_combo / _read_wheel_pi_blocking block."""
        vals = dict(self.get_combo())  # all PARAM_SPECS doubles, live
        pi = self._read_wheel_pi_blocking()
        if pi is not None:
            vals["wheel_pi_enabled"] = pi
        return self._persist_params_to_yaml(vals)

    @staticmethod
    def _fmt_yaml_scalar(v):
        if isinstance(v, bool):
            return "true" if v else "false"
        s = "%.6g" % float(v)
        if "." not in s and "e" not in s and "E" not in s:
            s += ".0"  # keep it a YAML float, not an int
        return s

    def _persist_params_to_yaml(self, values):
        """Line-splice ``values`` into mowgli_robot.yaml under mowgli/
        ros__parameters, preserving comments + layout (same approach as
        calibrate_imu_yaw_node / map_server set_docking_point). Keys already
        present are rewritten in place; missing keys (e.g. wheel_pid_deadband_pwm,
        which is absent by default) are appended into the block."""
        path = self.ROBOT_YAML
        if not os.path.isfile(path):
            return {"persisted": False, "error": f"{path} not found"}
        try:
            with open(path, "r") as f:
                lines = f.readlines()
        except OSError as e:
            return {"persisted": False, "error": str(e)}

        # Locate the `mowgli:` → `ros__parameters:` block, its indent, and its end
        # (the first line that dedents back to the ros__parameters level or above).
        ros_idx, ros_indent, in_mowgli = None, "", False
        for i, line in enumerate(lines):
            if re.match(r"^mowgli:\s*$", line):
                in_mowgli = True
                continue
            if in_mowgli:
                m = re.match(r"^(\s*)ros__parameters:\s*$", line)
                if m:
                    ros_idx, ros_indent = i, m.group(1)
                    break
        if ros_idx is None:
            return {"persisted": False,
                    "error": "could not locate mowgli/ros__parameters in yaml"}

        # DERIVE the child-key indent from an existing key in the block (this file
        # nests children at 8 spaces, the in-repo copy at 4). Hardcoding it
        # produced invalid YAML that crash-looped the launch, so never assume.
        key_re = re.compile(r"^(\s*)([A-Za-z0-9_]+)(\s*:\s*)([^#\n]*?)(\s*#.*)?\s*$")
        child_indent, block_end = None, len(lines)
        for j in range(ros_idx + 1, len(lines)):
            ln = lines[j]
            if not ln.strip() or ln.lstrip().startswith("#"):
                continue
            indent = ln[: len(ln) - len(ln.lstrip())]
            if len(indent) <= len(ros_indent):  # dedent → end of the block
                block_end = j
                break
            if child_indent is None:
                child_indent = indent
        if child_indent is None:
            child_indent = ros_indent + "  "  # empty block → one level deeper

        # Rewrite keys already present (within the block) in place.
        remaining = dict(values)
        written = {}
        for i in range(ros_idx + 1, block_end):
            m = key_re.match(lines[i])
            if not m or m.group(2) not in remaining:
                continue
            key, comment = m.group(2), (m.group(5) or "")
            vs = self._fmt_yaml_scalar(remaining.pop(key))
            lines[i] = f"{m.group(1)}{key}: {vs}{comment}\n"
            written[key] = vs

        # Append still-missing keys at the end of the block, at the derived indent.
        if remaining:
            block = [f"{child_indent}{k}: {self._fmt_yaml_scalar(v)}  # set by drive_tuning_node\n"
                     for k, v in remaining.items()]
            lines[block_end:block_end] = block
            for k, v in remaining.items():
                written[k] = self._fmt_yaml_scalar(v)

        try:
            with open(path, "w") as f:
                f.writelines(lines)
        except OSError as e:
            return {"persisted": False, "error": str(e), "written": written}
        return {"persisted": True, "path": path, "written": written}

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
            self.create_subscription(WheelTick, "/wheel_ticks", self._on_ticks, self._rel),
        ]
        self.armed = True
        self._refresh_live_params()
        self._read_saved_params()
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

    def drive_straight_open_loop(self, dist, speed):
        """Drive straight ``dist`` m at constant ``speed`` by publishing cmd_vel
        (v=speed, w=0) directly to the twist_mux teleop input — NO RPP, NO Smac.

        This is the instrument for the open-loop wheel-feedforward ID steps
        (breakaway, viscous). With wheel PI off, the linear channel is pure
        open-loop feedforward (the thing those steps identify), while the
        hardware_bridge gyro angular-rate loop holds heading straight. Tracking
        a Smac path with RPP instead (the old path) made RPP's short-lookahead
        heading loop fight the deadband-gated open-loop plant: it lagged ~1 s
        and limit-cycled into a ±0.4 rad/s weave that ALSO contaminated the
        feedforward measurement (the wheels ran differentially the whole run).

        Stops on reaching ``dist`` (odom euclidean from the start), on timeout,
        or on cancel. Captures samples (self.recording) for the metrics."""
        if self.x is None:
            return "NO_ODOM"
        sx, sy = self.x, self.y
        self.samples = []
        self.recording = True
        period = 0.05  # 20 Hz, well inside the twist_mux teleop 0.5 s timeout
        expected = dist / max(speed, 0.05)
        timeout = expected * 2.0 + 4.0
        t0 = time.time()
        reached = False
        try:
            while rclpy.ok() and not self._stop:
                if time.time() - t0 > timeout:
                    break
                if self.x is not None and math.hypot(self.x - sx, self.y - sy) >= dist:
                    reached = True
                    break
                m = TwistStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = "base_link"
                m.twist.linear.x = float(speed)
                m.twist.angular.z = 0.0
                self.pub_teleop.publish(m)
                time.sleep(period)
        finally:
            # Explicit zeros so the firmware halts instead of coasting on the
            # last command (the drive base has no holding torque at zero cmd).
            for _ in range(12):
                z = TwistStamped()
                z.header.stamp = self.get_clock().now().to_msg()
                z.header.frame_id = "base_link"
                self.pub_teleop.publish(z)
                time.sleep(0.04)
            self.recording = False
        if self._stop:
            return "CANCEL"
        return "OK" if reached else "TIMEOUT"

    def breakaway_staircase(self):
        """Find the static-friction breakaway PWM by ramping a pure-forward
        command from rest and watching EACH drive wheel independently.

        Why not the optimizer: the firmware breakaway is an ADDITIVE PWM offset
        (pwm = pwm_per_mps*v + sign(v)*deadband). At any cruise speed the
        pwm_per_mps*v term swamps it, so achieved speed is ~insensitive to the
        deadband and coordinate descent gets no gradient — it just retries
        values. The physical breakaway is the minimum PWM that overcomes static
        friction FROM REST; that needs a staircase, not a scored cruise.

        Method (single shared firmware deadband, no firmware change): temporarily
        zero the deadband, disable the min-vel clamp and the angular-rate loop
        (wheel PI is already off for this open-loop step), then ramp the
        commanded forward velocity from ~0. With those off the per-wheel PWM is
        exactly pwm_per_mps*v. At each rung, hold briefly and check each rear
        wheel's tick delta + the gyro: the lower-friction wheel breaks away
        first (the robot yaws), the stiffer wheel second. Record each wheel's
        breakaway PWM and set the single deadband to max(L,R)+margin so BOTH
        wheels reliably start. Restores the borrowed params; sets the deadband."""
        if self.x is None:
            return {"error": "no localization"}
        ppm = self.get_combo().get("wheel_pid_pwm_per_mps", 300.0)
        if ppm <= 1.0:
            return {"error": "pwm_per_mps too small to run the staircase"}

        # Borrow: zero the deadband (so PWM == pwm_per_mps*v), drop the host
        # min-vel clamp (we probe sub-clamp speeds), disable the gyro angular
        # loop (it would fight the asymmetric yaw and mask which wheel moved).
        saved = self._get_param_values(["min_linear_vel", "angular_rate_loop_enabled"])
        self.set_param(HB, "wheel_pid_deadband_pwm", 0.0, ParameterType.PARAMETER_DOUBLE)
        self.set_param(HB, "min_linear_vel", 0.0, ParameterType.PARAMETER_DOUBLE)
        self.set_param(HB, "angular_rate_loop_enabled", False, ParameterType.PARAMETER_BOOL)
        time.sleep(0.4)  # let the bridge push the zeroed deadband to the STM32

        PWM_LO, PWM_HI, PWM_STEP = 6.0, 120.0, 4.0
        DWELL_S = 1.0
        TICKS_MOVED = 3           # encoder counts in the dwell = wheel turned
        MARGIN_PWM = 6.0          # headroom above the stiffer wheel's breakaway
        bk_l = bk_r = None
        chosen = None
        rungs = []
        try:
            pwm = PWM_LO
            while pwm <= PWM_HI and not self._stop and (bk_l is None or bk_r is None):
                v = pwm / ppm
                rl0, rr0 = self._ticks_rl, self._ticks_rr
                gz_peak = 0.0
                t0 = time.time()
                while time.time() - t0 < DWELL_S and not self._stop:
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "base_link"
                    msg.twist.linear.x = float(v)
                    msg.twist.angular.z = 0.0
                    self.pub_teleop.publish(msg)
                    gz_peak = max(gz_peak, abs(self._gz))
                    time.sleep(0.05)
                dl = abs(self._ticks_rl - rl0)
                dr = abs(self._ticks_rr - rr0)
                moved_l, moved_r = dl >= TICKS_MOVED, dr >= TICKS_MOVED
                if bk_l is None and moved_l:
                    bk_l = pwm
                if bk_r is None and moved_r:
                    bk_r = pwm
                rungs.append({"pwm": pwm, "v": round(v, 4), "d_rl": dl, "d_rr": dr,
                              "gz_peak": round(gz_peak, 3),
                              "moved_l": moved_l, "moved_r": moved_r})
                self._set_phase(
                    f"breakaway staircase PWM={pwm:.0f} "
                    f"L={'Y' if (bk_l is not None) else '-'} "
                    f"R={'Y' if (bk_r is not None) else '-'}")
                pwm += PWM_STEP
        finally:
            # Stop the wheels (explicit zeros — no holding torque at zero cmd).
            for _ in range(12):
                z = TwistStamped()
                z.header.stamp = self.get_clock().now().to_msg()
                z.header.frame_id = "base_link"
                self.pub_teleop.publish(z)
                time.sleep(0.04)
            # Restore borrowed params; SET the discovered deadband.
            if "min_linear_vel" in saved:
                self.set_param(HB, "min_linear_vel", saved["min_linear_vel"],
                               ParameterType.PARAMETER_DOUBLE)
            if "angular_rate_loop_enabled" in saved:
                self.set_param(HB, "angular_rate_loop_enabled",
                               saved["angular_rate_loop_enabled"], ParameterType.PARAMETER_BOOL)
            found = [b for b in (bk_l, bk_r) if b is not None]
            if found:
                chosen = max(found) + MARGIN_PWM
                self.set_param(HB, "wheel_pid_deadband_pwm", float(chosen),
                               ParameterType.PARAMETER_DOUBLE)
            time.sleep(0.3)

        if self._stop:
            return {"follow_status": "CANCEL", "rungs": rungs,
                    "breakaway_left_pwm": bk_l, "breakaway_right_pwm": bk_r}
        return {
            "breakaway_left_pwm": bk_l, "breakaway_right_pwm": bk_r,
            "deadband_set_pwm": chosen, "margin_pwm": MARGIN_PWM,
            "pwm_per_mps": ppm, "rungs": rungs,
            "follow_status": "OK" if (bk_l is not None and bk_r is not None) else "INCOMPLETE",
        }

    def _read_operating_speeds(self):
        """Read mowing_speed / transit_speed from mowgli_robot.yaml so the
        viscous fit calibrates at the REAL operating range instead of a fixed
        guess. Defaults match the firmware/site config (0.2 / 0.25 m/s)."""
        speeds = {"mowing_speed": 0.2, "transit_speed": 0.25}
        try:
            with open(self.ROBOT_YAML) as f:
                text = f.read()
        except OSError:
            return speeds
        for key in speeds:
            m = re.search(rf"^\s*{key}\s*:\s*([0-9.]+)", text, re.M)
            if m:
                try:
                    speeds[key] = float(m.group(1))
                except ValueError:
                    pass
        return speeds

    def viscous_fit(self):
        """Identify the feedforward gain wheel_pid_pwm_per_mps by a MULTI-POINT
        fit across the operating speed range — not a single point.

        Drives a straight open-loop command (PI off, breakaway already set by
        step 1) at N speeds bracketing the configured mowing/transit speed. At
        each it records the achieved forward speed (wheel odom) and the applied
        feedforward PWM (= deadband + pwm_per_mps*v_cmd; PI is off so this IS the
        real applied PWM). The plant's steady state obeys PWM = deadband_true +
        slope_true*v_act, so a least-squares fit of PWM vs achieved speed
        recovers the TRUE gain (slope) and deadband (intercept) regardless of the
        current — possibly wrong — settings. The fit's r2 also MEASURES whether
        the affine model holds over the range instead of assuming a single
        low-speed point extrapolates linearly to the operating speed."""
        if self.x is None:
            return {"error": "no localization"}
        combo = self.get_combo()
        d_set = combo.get("wheel_pid_deadband_pwm")
        s_set = combo.get("wheel_pid_pwm_per_mps")
        if d_set is None or s_set is None:
            return {"error": "could not read current deadband / pwm_per_mps from /hardware_bridge"}

        op = self._read_operating_speeds()
        v_op_hi = max(op["mowing_speed"], op["transit_speed"])
        # Bracket the operating range: a bit below the slow end up to just above
        # the fast end, so the fit covers where the robot actually drives.
        v_hi = min(0.6, v_op_hi * 1.2)
        v_lo = max(0.10, v_hi * 0.45)
        n_pts = 4  # 4 points → a robust slope AND enough residual DOF that a low
        #            r2 actually means curvature, not just a perfect 2-3 pt line
        speeds = [v_lo + (v_hi - v_lo) * k / (n_pts - 1) for k in range(n_pts)]
        # ~2 s dwell keeps total forward travel (Σ v·dwell) inside the ~2.5 m
        # clearance; 0.7 s settle drops the per-point accel transient.
        DWELL_S, SETTLE_S = 2.0, 0.7

        # Drop the host min-vel clamp so low test speeds aren't zeroed. The gyro
        # angular loop can stay on (keeps it straight); its corrections are
        # zero-mean across the wheels, so the MEAN PWM == the feedforward formula.
        saved = self._get_param_values(["min_linear_vel"])
        self.set_param(HB, "min_linear_vel", 0.0, ParameterType.PARAMETER_DOUBLE)
        time.sleep(0.3)
        points = []
        try:
            for v_cmd in speeds:
                if self._stop:
                    break
                vacc = []
                t0 = time.time()
                while time.time() - t0 < DWELL_S and not self._stop:
                    m = TwistStamped()
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.header.frame_id = "base_link"
                    m.twist.linear.x = float(v_cmd)
                    m.twist.angular.z = 0.0
                    self.pub_teleop.publish(m)
                    if time.time() - t0 > SETTLE_S:  # skip the accel transient
                        vacc.append(self._wheel_v)
                    time.sleep(0.05)
                v_act = sum(vacc) / len(vacc) if vacc else 0.0
                pwm_applied = d_set + s_set * v_cmd  # PI off → real applied PWM
                points.append({"v_cmd": round(v_cmd, 3), "v_act": round(v_act, 3),
                               "pwm": round(pwm_applied, 1)})
                self._set_phase(f"viscous fit v_cmd={v_cmd:.2f} v_act={v_act:.2f}")
        finally:
            for _ in range(12):  # stop
                z = TwistStamped()
                z.header.stamp = self.get_clock().now().to_msg()
                z.header.frame_id = "base_link"
                self.pub_teleop.publish(z)
                time.sleep(0.04)
            if "min_linear_vel" in saved:
                self.set_param(HB, "min_linear_vel", saved["min_linear_vel"],
                               ParameterType.PARAMETER_DOUBLE)
            time.sleep(0.2)

        if self._stop:
            return {"follow_status": "CANCEL", "points": points}
        fit = _linfit([p["v_act"] for p in points], [p["pwm"] for p in points])
        if fit is None or fit[0] <= 0.0:
            return {"follow_status": "INCOMPLETE", "points": points,
                    "error": "fit failed — need >=2 distinct achieved speeds and a positive slope"}
        slope, intercept, r2 = fit
        lo, hi = PARAM_SPECS["wheel_pid_pwm_per_mps"][1], PARAM_SPECS["wheel_pid_pwm_per_mps"][2]
        new_ppm = max(lo, min(hi, slope))
        ok = self.set_param(HB, "wheel_pid_pwm_per_mps", float(new_ppm),
                            ParameterType.PARAMETER_DOUBLE)
        time.sleep(0.3)
        return {
            "points": points,
            "pwm_per_mps_fit": slope, "pwm_per_mps_set": new_ppm,
            "implied_deadband_pwm": intercept, "deadband_used": d_set,
            "linearity_r2": r2, "nonlinear_warning": r2 < 0.95,
            "operating_speeds": op,
            "follow_status": "OK" if ok else "INCOMPLETE",
        }

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

    def metrics_open_loop(self, sx, sy, gx, gy, gyaw, cmd_v):
        """Open-loop straight-drive metrics: achieved forward speed (vs the
        commanded feedforward), peak lateral drift off the straight line, and
        the same final-pose error as the closed-loop transit. ``achieved_speed``
        is the wheel-odom mean over the cruise (first/last 0.5 s dropped so the
        breakaway ramp + the stop don't bias it)."""
        m = self.metrics_pose(gx, gy, gyaw)
        if self.samples:
            t0, t1 = self.samples[0][0], self.samples[-1][0]
            cruise = [s for s in self.samples if s[0] - t0 > 0.5 and t1 - s[0] > 0.5]
            wheel_vs = [s[7] for s in (cruise or self.samples)]
            achieved = sum(wheel_vs) / len(wheel_vs) if wheel_vs else 0.0
            line = [(sx, sy), (gx, gy)]
            drift = max((_cross_track(s[1], s[2], line) for s in self.samples), default=0.0)
        else:
            achieved, drift = 0.0, 0.0
        m["achieved_speed"] = achieved
        m["cmd_speed"] = cmd_v
        m["speed_err"] = abs(cmd_v - achieved)
        m["drift_peak"] = drift
        return m

    def score(self, maneuver, m):
        """Scalar objective (lower = better)."""
        g = self.get_parameter
        w_ct = g("w_cross_track").value
        w_h = g("w_heading").value
        w_hunt = g("w_hunting").value
        w_bc = g("w_backcreep").value
        w_pe = g("w_pose_err").value
        if maneuver in ("breakaway_staircase", "viscous_fit"):
            return 0.0  # not scored — these fit + set the value directly
        if maneuver == "yaw_hunt":
            return (w_h * m.get("undershoot", 0.0) + w_bc * m.get("backcreep", 0.0)
                    + w_hunt * 0.1 * m.get("gz_zc", 0.0))
        if maneuver == "transit_pose":
            # speed_err + drift_peak are only present for the open-loop straight
            # drive (breakaway/viscous); absent → 0, so closed-loop transit
            # (pid_trim/windup) scores exactly as before.
            return (w_pe * m.get("pose_xy_err", 0.0) + w_h * m.get("pose_yaw_err", 0.0)
                    + g("w_speed").value * m.get("speed_err", 0.0)
                    + w_ct * m.get("drift_peak", 0.0))
        # outline / swath: path tracking
        return (w_ct * m.get("ct_rms", 0.0) + w_h * m.get("herr_rms", 0.0)
                + w_hunt * 0.1 * m.get("wz_zc", 0.0))

    # ---- one maneuver run ---------------------------------------------------
    def run_maneuver(self, maneuver, open_loop=False):
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
                sx, sy = self.x, self.y
                gx, gy = self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw)
                gyaw = self.yaw
                if open_loop:
                    # Open-loop wheel-feedforward ID (breakaway/viscous): drive a
                    # straight cmd_vel, NOT an RPP-tracked Smac path — RPP on the
                    # PI-off plant limit-cycles into a weave (see
                    # drive_straight_open_loop).
                    speed = float(self.get_parameter("open_loop_speed_mps").value)
                    st = self.drive_straight_open_loop(d, speed)
                    m = self.metrics_open_loop(sx, sy, gx, gy, gyaw, speed)
                else:
                    path = self.plan_smac(gx, gy, gyaw)
                    if path is None:
                        return {"error": "Smac plan failed"}
                    st = self.follow(path, "FollowPath", "stopped_goal_checker")
                    m = self.metrics_pose(gx, gy, gyaw)
            elif maneuver == "breakaway_staircase":
                m = self.breakaway_staircase()
                if "error" in m:
                    return m
                st = m.get("follow_status", "OK")
            elif maneuver == "viscous_fit":
                m = self.viscous_fit()
                if "error" in m:
                    return m
                st = m.get("follow_status", "OK")
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

    def _return_to_start(self, start, open_loop=False):
        """Drive back to the pose captured at the start of optimize() so the
        descent's footprint stays bounded (~one maneuver length) no matter how
        many iterations run, instead of marching cumulatively forward. Returns
        False if it cannot get back (so the descent aborts rather than walking
        off the cleared area). A clean user stop counts as success.

        The return leg is NOT a scored measurement, so for the open-loop ID
        steps it re-enables wheel PI for the RPP drive back: with PI on the
        wheels track the heading loop fast (no open-loop weave) AND the closed
        loop keeps the footprint accurately bounded — then PI is restored to off
        for the next scored run."""
        if self._stop:
            return True
        if self.x is None:
            return False
        sx, sy, syaw = start
        self._set_phase("returning to start")
        dist = math.hypot(sx - self.x, sy - self.y)
        pi_restored = False
        if open_loop and dist > 0.08:
            # Closed-loop for the return only — RPP would weave on the PI-off plant.
            self.set_param(HB, "wheel_pi_enabled", True, ParameterType.PARAMETER_BOOL)
            time.sleep(0.3)  # let the bridge push the toggle to firmware
            pi_restored = True
        try:
            # 1) Translate back along the same corridor if we moved. RotationShim
            #    pivots ~180 deg in place, RPP drives the straight path back.
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
        finally:
            if pi_restored:
                # Back to PI-off so the next scored ID run measures feedforward.
                self.set_param(HB, "wheel_pi_enabled", False, ParameterType.PARAMETER_BOOL)
                time.sleep(0.3)

    # ---- coordinate / finite-difference descent -----------------------------
    def optimize(self, maneuver, param_names=None, open_loop=False):
        max_iters = int(self.get_parameter("optimize_max_iters").value)
        # param_names lets a protocol step optimize only its own subset; the rest
        # of the combo is read + reapplied unchanged. Default = sweep everything.
        names = list(param_names) if param_names else list(PARAM_SPECS.keys())
        combo = self.get_combo()
        # get_combo() now returns only params it could actually read. If the
        # current values can't be read, abort — re-applying a partial combo (or
        # descending on a missing key) is how gains used to get zeroed.
        missing = [n for n in names if n not in combo]
        if missing:
            return {"error": "could not read current drive params "
                             f"({', '.join(missing)}) from /hardware_bridge — aborting"}
        steps = {n: PARAM_SPECS[n][3] for n in names}
        self.apply_params(combo)
        # Anchor the footprint: every scored run returns here, so the descent
        # needs only ~one maneuver of clearance regardless of iteration count.
        if self.x is None:
            return {"error": "no localization — cannot anchor return-to-start"}
        start = (self.x, self.y, self.yaw)
        base = self.run_maneuver(maneuver, open_loop=open_loop)
        if "error" in base:
            return base
        if not self._return_to_start(start, open_loop=open_loop):
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
                res = self.run_maneuver(maneuver, open_loop=open_loop)
                if "error" in res:
                    return res
                history.append({"combo": dict(cand), "metrics": res, "score": res["score"]})
                if not self._return_to_start(start, open_loop=open_loop):
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
        # Refresh the live param values + saved baseline ~every 2 s (outside the
        # lock, and only while idle so a campaign's transient per-step values
        # aren't displayed) so the GUI panel reflects test results + any
        # Drive-Motor-settings change without needing a re-arm.
        self._pi_poll_tick += 1
        if self._pi_poll_tick % 2 == 1 and not self._busy:
            self._refresh_live_params()
            self._read_saved_params()
        with self._lock:
            snap = dict(self._status)
        snap["busy"] = self._busy
        snap["armed"] = self.armed
        snap["wheel_pi_enabled"] = self.wheel_pi_enabled
        snap["hl_state"] = self.hl_state
        snap["sigma_xy"] = self.sx
        snap["params"] = self._build_params_panel()
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
        if action == "persist":
            # Write the live tuned drive params to mowgli_robot.yaml. Allowed
            # while idle (not armed) but never mid-campaign.
            if self._busy:
                self.get_logger().warn("busy — ignoring persist")
                return
            threading.Thread(target=self._persist_thread, daemon=True).start()
            return
        if action == "set_param":
            # Manually set one tunable's LIVE value (Edit button in the panel).
            if self._busy:
                self.get_logger().warn("busy — ignoring set_param")
                return
            threading.Thread(target=self._set_param_thread,
                             args=(cmd.get("name"), cmd.get("value")), daemon=True).start()
            return
        if action == "reset_default":
            # Restore Live = Default (saved-config or firmware default) for one
            # param (name given) or all (no name). The recovery for zeroed gains.
            if self._busy:
                self.get_logger().warn("busy — ignoring reset_default")
                return
            threading.Thread(target=self._reset_default_thread,
                             args=(cmd.get("name"),), daemon=True).start()
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
        pi_run = None  # None = leave PI at the operating mode; True/False = force it
        if action in ("run_step", "optimize_step"):
            step = self._find_step(cmd.get("step"))
            if step is None:
                self.get_logger().warn(f"unknown protocol step: {cmd.get('step')!r}")
                return
            maneuver = step["maneuver"]
            params = step["params"]
            pi_spec = step.get("pi_run")
            if pi_spec == "off":
                pi_run = False
            elif pi_spec == "on":
                pi_run = True
            action = "optimize" if action == "optimize_step" else "run"
        else:
            maneuver = cmd.get("maneuver", self.get_parameter("maneuver").value)
        if not self.armed:
            self.get_logger().warn("not armed — send start_session before running maneuvers")
            self._set_phase("not armed — press Start tuning")
            return
        threading.Thread(target=self._campaign, args=(action, maneuver, params, pi_run),
                         daemon=True).start()

    def _persist_thread(self):
        self._busy = True  # serialize against campaigns; status shows "running"
        try:
            self._set_phase("saving to mowgli_robot.yaml")
            res = self.persist_params()
            self._set_phase("saved to config" if res.get("persisted")
                            else "save to config failed", persist=res)
            self.get_logger().info(f"persist: {json.dumps(res, default=str)}")
        finally:
            self._busy = False

    def _set_param_thread(self, name, value):
        """Apply one tunable's live value on /hardware_bridge (manual Edit).
        Doubles are clamped to their PARAM_SPECS range; wheel_pi_enabled is a
        bool. The status timer re-reads the live value once _busy clears."""
        self._busy = True
        try:
            if name in PARAM_SPECS:
                lo, hi = PARAM_SPECS[name][1], PARAM_SPECS[name][2]
                try:
                    v = max(lo, min(hi, float(value)))
                except (TypeError, ValueError):
                    self._set_phase(f"set {name}: bad value {value!r}")
                    return
                ok = self.set_param(HB, name, v, ParameterType.PARAMETER_DOUBLE)
                shown = v
            elif name == "wheel_pi_enabled":
                ok = self.set_param(HB, name, bool(value), ParameterType.PARAMETER_BOOL)
                shown = bool(value)
            else:
                self._set_phase(f"set_param: unknown param {name!r}")
                return
            time.sleep(0.3)  # let the bridge push the value to the STM32
            self._set_phase(f"set {name} = {shown}" if ok else f"set {name} failed")
            self.get_logger().info(f"set_param {name} = {shown}: {'ok' if ok else 'FAILED'}")
        finally:
            self._busy = False

    def _reset_default_thread(self, name=None):
        """Set Live = Default (the from-scratch value: mowgli_robot.yaml if the
        key is persisted, else the firmware default) for one param or all. This
        is the recovery path when a value has drifted/zeroed from its config."""
        self._busy = True
        try:
            self._read_saved_params()  # freshest saved baseline
            targets = [name] if name else list(PANEL_PARAM_NAMES)
            applied = {}
            for n in targets:
                if n not in PANEL_PARAM_NAMES:
                    continue
                default = (self._saved_params[n] if n in self._saved_params
                           else FIRMWARE_DEFAULTS.get(n))
                if default is None:
                    continue
                if n == "wheel_pi_enabled":
                    ok = self.set_param(HB, n, bool(default), ParameterType.PARAMETER_BOOL)
                else:
                    ok = self.set_param(HB, n, float(default), ParameterType.PARAMETER_DOUBLE)
                if ok:
                    applied[n] = default
            time.sleep(0.3)  # let the bridge push the values to the STM32
            self._set_phase(f"reset {len(applied)} param(s) to default", reset=applied)
            self.get_logger().info(f"reset_default: {json.dumps(applied, default=str)}")
        finally:
            self._busy = False

    def _find_step(self, step_id):
        return next((s for s in PROTOCOL if s["id"] == step_id), None)

    def _campaign(self, action, maneuver, params=None, pi_run=None):
        self._busy = True
        self._stop = False
        operating_pi = None
        try:
            # Open-loop ID vs closed-loop refinement: the feedforward steps must
            # run with PI off (the integrator masks breakaway / speed-scale
            # error), the PI steps with it on. Capture the operating mode so we
            # can restore it, then force the mode this step needs.
            if pi_run is not None:
                operating_pi = self._read_wheel_pi_blocking()
                if operating_pi is None:
                    operating_pi = bool(self.wheel_pi_enabled) \
                        if self.wheel_pi_enabled is not None else True
                if pi_run != operating_pi:
                    self.set_param(HB, "wheel_pi_enabled", pi_run,
                                   ParameterType.PARAMETER_BOOL)
                    time.sleep(0.3)  # let the bridge push the toggle to firmware
            # The breakaway staircase and the viscous multi-point fit set their
            # param directly (PWM ramp / regression); they have no scored
            # gradient, so never wrap them in the coordinate-descent optimizer.
            if maneuver in ("breakaway_staircase", "viscous_fit"):
                action = "run"
            # Open-loop ID steps (pi_run forced off) drive a straight cmd_vel
            # instead of an RPP-tracked path — RPP on the PI-off plant weaves.
            open_loop = (pi_run is False)
            if action == "optimize":
                self._set_phase(f"optimizing {maneuver}", params=params)
                result = self.optimize(maneuver, params, open_loop=open_loop)
            else:
                self._set_phase(f"running {maneuver}")
                result = self.run_maneuver(maneuver, open_loop=open_loop)
            self._set_phase(f"done {maneuver}", result=result)
            self.get_logger().info(f"{action} {maneuver}: {json.dumps(result, default=str)}")
        finally:
            # Restore the operating PI mode BEFORE clearing _busy so the status
            # poll never reads the transient per-step value.
            if pi_run is not None and operating_pi is not None and pi_run != operating_pi:
                self.set_param(HB, "wheel_pi_enabled", operating_pi,
                               ParameterType.PARAMETER_BOOL)
            # Leave RECORDING so the BT returns to idle cleanly.
            self.hl(CMD_RECORD_CANCEL)
            self._busy = False


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
