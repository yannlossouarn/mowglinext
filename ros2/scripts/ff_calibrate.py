#!/usr/bin/env python3
"""Drive friction-feedforward calibration assistant (interactive TUI).

Runs inside the mowgli-ros2 container. Lets you set the firmware's open-loop
friction feedforward live (Coulomb fc / viscous fv / breakaway fb), trigger
short directional drive tests (forward / backward / rotate left / rotate
right), read the motion back from BOTH the wheel encoders (/wheel_odom) and the
chassis IMU (/imu/data), get a proposal ("did the wheels break free? raise or
lower fb?"), and reconcile it against what you physically observed.

    Push     : ROS params on /hardware_bridge — ff_coulomb_byte /
               ff_viscous_byte_per_mps / ff_breakaway_byte
               (NaN = firmware default, three-tier precedence).
    Readback : /hardware_bridge/drive_cal_status (firmware passive estimate).
    Drive    : TwistStamped on /cmd_vel_teleop @ 20 Hz (fw watchdog 200 ms).

PREREQUISITES (the header flags these live):
  * Firmware ignores cmd_vel in IDLE -> put the GUI in MANUAL MOWING (HL
    state 4) so the wheels will actually move.
  * /wheel_odom is force-zeroed while CHARGING -> lift the robot off the dock
    contacts (the IMU is unaffected).
  * This tool NEVER commands the blade. Turn the blade off in the GUI.

Usage (from the host — ROS must be sourced, which a bare `docker exec
python3 ...` does NOT do, hence the bash -lc + source):
    ./docker/ff_calibrate.sh                      # convenience launcher
  or:
    docker exec -it mowgli-ros2 bash -lc \\
      'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \\
       exec python3 /ros2_ws/maps/ff_calibrate.py'
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mowgli_interfaces.msg import DriveCalStatus, Emergency, HighLevelStatus, Status

WB = 0.325  # wheel track [m] (half-track = 0.1625)
WHEEL_MOVE_MPS = 0.03  # |wheel velocity| above which we call it "moving"
GYRO_MOVE_RPS = 0.05  # |gyro_z| above which the chassis is "rotating"
ACCEL_MOVE_MSS = 0.30  # horizontal accel deviation -> chassis "translating"

HL_NAMES = {0: "NULL", 1: "IDLE", 2: "AUTONOMOUS", 3: "RECORDING", 4: "MANUAL_MOWING"}
HL_DRIVES = {2, 3, 4}  # firmware accepts cmd_vel out of IDLE/NULL


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


class CalNode(Node):
    def __init__(self):
        super().__init__("ff_calibrate")
        rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        q10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", 10)
        self.create_subscription(Odometry, "/wheel_odom", self._on_odom, rel)
        self.create_subscription(Imu, "/imu/data", self._on_imu, q10)
        self.create_subscription(DriveCalStatus, "/hardware_bridge/drive_cal_status",
                                 self._on_cal, q10)
        self.create_subscription(Emergency, "/hardware_bridge/emergency", self._on_emg, q10)
        self.create_subscription(HighLevelStatus, "/behavior_tree_node/high_level_status",
                                 self._on_hl, q10)
        self.create_subscription(Status, "/hardware_bridge/status", self._on_status, q10)
        self.param_cli = self.create_client(SetParameters, "/hardware_bridge/set_parameters")

        # live state
        self.meas_vx = self.meas_wz = 0.0
        self.gyro_z = self.acc_x = self.acc_y = 0.0
        self.cal = None
        self.cal_t = 0.0
        self.emergency = False
        self.hl_state = None
        self.is_charging = None
        self.status_t = 0.0

        # pushed FF (mirror of what we set; NaN = unset/default)
        self.ff = {"coulomb": float("nan"), "viscous": float("nan"), "breakaway": float("nan")}

        # drive command + collection
        self.cmd_vx = self.cmd_wz = 0.0
        self._drive_active = False
        self._collect = False
        self._reset_peaks()
        self.create_timer(0.05, self._tick)  # 20 Hz keepalive

    # ---- callbacks -------------------------------------------------------
    def _on_odom(self, m):
        self.meas_vx = m.twist.twist.linear.x
        self.meas_wz = m.twist.twist.angular.z
        if self._collect:
            vl = self.meas_vx - self.meas_wz * WB / 2.0
            vr = self.meas_vx + self.meas_wz * WB / 2.0
            self.pk_l = max(self.pk_l, abs(vl))
            self.pk_r = max(self.pk_r, abs(vr))
            self.pk_odom_wz = max(self.pk_odom_wz, abs(self.meas_wz))

    def _on_imu(self, m):
        self.gyro_z = m.angular_velocity.z
        self.acc_x = m.linear_acceleration.x
        self.acc_y = m.linear_acceleration.y
        if self._collect:
            self.pk_gyro = max(self.pk_gyro, abs(self.gyro_z - self.base_gyro))
            dev = math.hypot(self.acc_x - self.base_ax, self.acc_y - self.base_ay)
            self.pk_acc = max(self.pk_acc, dev)

    def _on_cal(self, m):
        self.cal = m
        self.cal_t = time.time()

    def _on_emg(self, m):
        self.emergency = bool(m.active_emergency or m.latched_emergency)

    def _on_hl(self, m):
        self.hl_state = m.state

    def _on_status(self, m):
        self.is_charging = m.is_charging
        self.status_t = time.time()

    # ---- drive -----------------------------------------------------------
    def _tick(self):
        if not self._drive_active:
            return
        if self.emergency:
            self.cmd_vx = self.cmd_wz = 0.0
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = self.cmd_vx
        t.twist.angular.z = self.cmd_wz
        self.pub.publish(t)

    def _reset_peaks(self):
        self.pk_l = self.pk_r = self.pk_odom_wz = 0.0
        self.pk_gyro = self.pk_acc = 0.0
        self.base_gyro = self.base_ax = self.base_ay = 0.0

    def run_test(self, vx, wz, hold, settle=0.6):
        """Command (vx, wz) for `hold` s, return peak motion metrics."""
        # baseline IMU at rest (gravity + bias) so deviations are meaningful
        self.base_gyro = self.gyro_z
        self.base_ax = self.acc_x
        self.base_ay = self.acc_y
        self._reset_peaks()
        self._collect = True
        self._drive_active = True
        self.cmd_vx, self.cmd_wz = vx, wz
        end = time.time() + hold
        while time.time() < end:
            if self.emergency:
                break
            left = end - time.time()
            sys.stdout.write(
                f"\r  driving... {left:4.1f}s  wheels L{self.pk_l:5.2f} R{self.pk_r:5.2f} m/s "
                f"| gyro {self.pk_gyro:5.2f} rad/s  ")
            sys.stdout.flush()
            time.sleep(0.05)
        self.cmd_vx = self.cmd_wz = 0.0  # stop; keep publishing zeros to settle
        time.sleep(settle)
        self._drive_active = False
        self._collect = False
        sys.stdout.write("\r" + " " * 78 + "\r")
        return {
            "peak_l": self.pk_l, "peak_r": self.pk_r, "odom_wz": self.pk_odom_wz,
            "gyro": self.pk_gyro, "accel": self.pk_acc,
        }

    def stop(self):
        self.cmd_vx = self.cmd_wz = 0.0
        self._drive_active = True
        for _ in range(8):
            self._tick()
            time.sleep(0.05)
        self._drive_active = False

    # ---- params ----------------------------------------------------------
    def set_ff(self, key, value):
        """Push one FF param (value may be NaN to clear). Returns (ok, msg)."""
        name = {"coulomb": "ff_coulomb_byte", "viscous": "ff_viscous_byte_per_mps",
                "breakaway": "ff_breakaway_byte"}[key]
        if not self.param_cli.wait_for_service(timeout_sec=3.0):
            return False, "param service unavailable"
        req = SetParameters.Request()
        p = ParamMsg()
        p.name = name
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [p]
        fut = self.param_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done() or not fut.result() or not fut.result().results:
            return False, "no response"
        res = fut.result().results[0]
        if res.successful:
            self.ff[key] = float(value)
            return True, "ok"
        return False, res.reason or "rejected"


# --------------------------------------------------------------------------
# UI helpers
# --------------------------------------------------------------------------
def fmt_ff(v):
    return c("default", Col.DIM) if math.isnan(v) else c(f"{v:.1f}", Col.B + Col.CYN)


def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        return "q"


def ask_yn(prompt):
    a = ask(f"  {prompt} [y/n/s=skip]: ").lower()
    return {"y": True, "n": False}.get(a[:1], None)


def header(node):
    clear()
    bar = c("=" * 64, Col.DIM)
    print(bar)
    print(c("  Drive Friction-Feedforward Calibration Assistant", Col.B + Col.MAG))
    print(bar)

    # link / prerequisites
    link_fresh = (time.time() - node.status_t) < 3.0 if node.status_t else False
    link = c("OK", Col.GRN) if link_fresh else c("NO DATA", Col.RED)
    print(f"  firmware link  : {link}")

    if node.is_charging is None:
        chg = c("unknown", Col.YEL)
    elif node.is_charging:
        chg = c("CHARGING -> /wheel_odom is force-zeroed! lift off dock", Col.RED)
    else:
        chg = c("not charging (odom live)", Col.GRN)
    print(f"  charging       : {chg}")

    st = node.hl_state
    name = HL_NAMES.get(st, "?")
    if st in HL_DRIVES:
        mode = c(f"{name} (accepts cmd_vel)", Col.GRN)
    elif st is None:
        mode = c("unknown", Col.YEL)
    else:
        mode = c(f"{name} -> firmware IDLE: put GUI in MANUAL MOWING", Col.RED)
    print(f"  firmware mode  : {mode}")

    emg = c("ACTIVE", Col.RED) if node.emergency else c("clear", Col.GRN)
    print(f"  emergency      : {emg}")

    print(bar)
    print("  Pushed FF override (host -> firmware):")
    print(f"    fc Coulomb   = {fmt_ff(node.ff['coulomb'])}   "
          f"fv Viscous = {fmt_ff(node.ff['viscous'])}   "
          f"fb Breakaway = {fmt_ff(node.ff['breakaway'])}  [byte]")
    if node.cal is not None and (time.time() - node.cal_t) < 5.0:
        cal = node.cal
        valid = c("valid", Col.GRN) if cal.est_valid else c("not converged", Col.DIM)
        host = c("HOST ACTIVE", Col.YEL) if cal.host_active else c("default/learned", Col.DIM)
        print("  Firmware estimate (drive_cal_status):")
        print(f"    est fc={cal.est_coulomb_byte:.1f}  fv={cal.est_viscous_byte_per_mps:.1f}  "
              f"fb={cal.est_breakaway_byte:.1f}  n={cal.sample_count}  {valid}  {host}")
    else:
        print(c("  Firmware estimate: no drive_cal_status (old firmware?)", Col.YEL))
    print(bar)


def speeds_line(s):
    return (f"  test speeds: linear {s['lin']:.2f} m/s | angular {s['ang']:.2f} rad/s "
            f"(edge {s['ang']*WB/2:.2f} m/s) | hold {s['hold']:.1f} s")


def proposal(node, test, m, speeds):
    """Build colored verdict + proposal lines from sensor metrics."""
    lines = []
    wheels = max(m["peak_l"], m["peak_r"]) > WHEEL_MOVE_MPS
    rotated = m["gyro"] > GYRO_MOVE_RPS
    translated = m["accel"] > ACCEL_MOVE_MSS
    charging = bool(node.is_charging)

    lines.append(c("  --- sensor readout ---", Col.B))
    wtxt = c(f"L {m['peak_l']:.3f}  R {m['peak_r']:.3f} m/s", Col.B)
    lines.append(f"  wheels (encoder) : {wtxt}  -> "
                 + (c("TURNED", Col.GRN) if wheels else c("still", Col.DIM)))
    lines.append(f"  chassis gyro_z   : {m['gyro']:.3f} rad/s   -> "
                 + (c("ROTATED", Col.GRN) if rotated else c("still", Col.DIM)))
    lines.append(f"  chassis accel    : {m['accel']:.2f} m/s^2    -> "
                 + (c("MOVED", Col.GRN) if translated else c("still", Col.DIM)))
    if charging:
        lines.append(c("  NOTE: charging -> wheel encoder is force-zeroed; trust gyro/accel "
                       "+ your eyes.", Col.YEL))

    lines.append(c("  --- proposal ---", Col.B))
    fb = node.ff["breakaway"]
    fbtxt = "default 55" if math.isnan(fb) else f"{fb:.0f}"
    fc = node.ff["coulomb"]
    fctxt = "default 35" if math.isnan(fc) else f"{fc:.0f}"

    moved = wheels or rotated or translated
    if node.emergency:
        lines.append(c("  EMERGENCY active -> clear it before trusting any result.", Col.RED))
    elif node.hl_state not in HL_DRIVES:
        lines.append(c(f"  Firmware not in a driving mode ({HL_NAMES.get(node.hl_state,'?')}). "
                       "cmd_vel is being dropped -> put the GUI in MANUAL MOWING.", Col.RED))
    elif moved:
        lines.append(c(f"  BROKE FREE at fc={fctxt}, fb={fbtxt} byte.", Col.GRN))
        lines.append("  -> The breakaway threshold is AT OR BELOW the current effort.")
        lines.append("     To find the floor: LOWER fb (e.g. -5 byte) and retest until it just")
        lines.append("     fails to move; the last value that still moved is your breakaway.")
        if wheels and not rotated and not translated:
            lines.append(c("  Wheels spin but the chassis is still -> free wheels on a STAND. "
                           "This is the no-load floor, NOT the on-ground breakaway.", Col.YEL))
    else:
        lines.append(c(f"  DID NOT MOVE at fc={fctxt}, fb={fbtxt} byte.", Col.RED))
        lines.append("  -> Effort was below breakaway. RAISE fb (e.g. +5 byte) and retest;")
        lines.append("     keep raising until it just breaks free.")
    return lines


def reconcile(node, m):
    """Ask the user what they saw and compare to the sensors."""
    saw_w = ask_yn("Did you SEE the wheels turn?")
    saw_r = ask_yn("Did the robot itself move / rotate?")
    s_w = max(m["peak_l"], m["peak_r"]) > WHEEL_MOVE_MPS
    s_r = (m["gyro"] > GYRO_MOVE_RPS) or (m["accel"] > ACCEL_MOVE_MSS)
    out = []
    for label, saw, sens, hint in (
        ("wheels", saw_w, s_w, "if charging, the encoder is force-zeroed; otherwise sub-threshold"),
        ("robot ", saw_r, s_r, "on a stand the chassis can't move even with wheels spinning"),
    ):
        if saw is None:
            continue
        if saw == sens:
            out.append(c(f"  [{label}] observation agrees with sensors "
                         f"(both {'moved' if saw else 'still'}).", Col.GRN))
        elif saw and not sens:
            out.append(c(f"  [{label}] you saw motion but sensors read still -> {hint}.", Col.YEL))
        else:
            out.append(c(f"  [{label}] sensors read motion but you saw none -> possible "
                         "encoder/odom glitch or wrong wheel watched.", Col.YEL))
    return out


def do_test(node, kind, speeds):
    arrows = {"fwd": ("FORWARD", speeds["lin"], 0.0),
              "back": ("BACKWARD", -speeds["lin"], 0.0),
              "left": ("ROTATE LEFT (CCW)", 0.0, speeds["ang"]),
              "right": ("ROTATE RIGHT (CW)", 0.0, -speeds["ang"])}
    title, vx, wz = arrows[kind]
    header(node)
    print(c(f"  TEST: {title}", Col.B + Col.BLU))
    if vx != 0.0 and abs(vx) < 0.15:
        print(c("  WARNING: |linear| < 0.15 m/s is zeroed by the bridge min_lin_vel clamp; "
                "raise the linear speed or use a rotate test for low-effort probing.", Col.YEL))
    if node.is_charging:
        print(c("  WARNING: robot is charging -> wheel encoder will read 0. Lift off the dock.",
                Col.YEL))
    if node.hl_state not in HL_DRIVES:
        print(c(f"  WARNING: firmware mode {HL_NAMES.get(node.hl_state,'?')} won't accept cmd_vel. "
                "Put the GUI in MANUAL MOWING first.", Col.YEL))
    print(f"  command: vx={vx:+.2f} m/s  wz={wz:+.2f} rad/s  for {speeds['hold']:.1f} s")
    a = ask("  Press ENTER to run (watch the wheels), or 'c' to cancel: ").lower()
    if a == "c":
        return
    for n in (3, 2, 1):
        sys.stdout.write(f"\r  starting in {n}... ")
        sys.stdout.flush()
        time.sleep(1.0)
    print()
    m = node.run_test(vx, wz, speeds["hold"])
    print()
    for ln in proposal(node, kind, m, speeds):
        print(ln)
    print()
    for ln in reconcile(node, m):
        print(ln)
    ask(c("\n  ENTER to return to menu...", Col.DIM))


def set_param_flow(node, key, label):
    header(node)
    cur = node.ff[key]
    curtxt = "default (NaN)" if math.isnan(cur) else f"{cur:.1f}"
    print(c(f"  Set {label} (current: {curtxt}) [byte]", Col.B))
    print(c("  Enter a number, 'd' for firmware default (NaN), or blank to cancel.", Col.DIM))
    a = ask("  value: ").strip().lower()
    if a == "":
        return
    val = float("nan") if a == "d" else None
    if val is None:
        try:
            val = float(a)
        except ValueError:
            print(c("  not a number", Col.RED))
            time.sleep(1.0)
            return
    ok, msg = node.set_ff(key, val)
    print(c(f"  {'pushed' if ok else 'FAILED'}: {label} = "
            f"{'default' if math.isnan(val) else val}  ({msg})", Col.GRN if ok else Col.RED))
    time.sleep(1.2)


def set_speeds_flow(node, speeds):
    header(node)
    print(c("  Test speeds", Col.B))
    print(speeds_line(speeds))
    for key, label, lo, hi in (("lin", "linear m/s", 0.05, 0.5),
                               ("ang", "angular rad/s", 0.1, 2.5),
                               ("hold", "hold seconds", 1.0, 10.0)):
        a = ask(f"  {label} [{speeds[key]:.2f}] (blank=keep): ").strip()
        if a:
            try:
                speeds[key] = max(lo, min(hi, float(a)))
            except ValueError:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.parse_args()
    rclpy.init()
    node = CalNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    time.sleep(0.8)
    speeds = {"lin": 0.18, "ang": 0.70, "hold": 4.0}
    try:
        while True:
            header(node)
            print(speeds_line(speeds))
            print()
            print("   1) Forward test      5) Set Coulomb  fc")
            print("   2) Backward test     6) Set Viscous  fv")
            print("   3) Rotate LEFT       7) Set Breakaway fb")
            print("   4) Rotate RIGHT      8) Clear ALL to firmware default")
            print("   r) Refresh          9) Set test speeds")
            print("   s) STOP (zero motors)   q) Quit")
            ch = ask("\n  choice: ").lower()
            if ch == "1":
                do_test(node, "fwd", speeds)
            elif ch == "2":
                do_test(node, "back", speeds)
            elif ch == "3":
                do_test(node, "left", speeds)
            elif ch == "4":
                do_test(node, "right", speeds)
            elif ch == "5":
                set_param_flow(node, "coulomb", "Coulomb fc")
            elif ch == "6":
                set_param_flow(node, "viscous", "Viscous fv")
            elif ch == "7":
                set_param_flow(node, "breakaway", "Breakaway fb")
            elif ch == "8":
                for k, lbl in (("coulomb", "fc"), ("viscous", "fv"), ("breakaway", "fb")):
                    node.set_ff(k, float("nan"))
                print(c("  all FF overrides cleared to firmware default (NaN)", Col.GRN))
                time.sleep(1.2)
            elif ch == "9":
                set_speeds_flow(node, speeds)
            elif ch == "s":
                node.stop()
                print(c("  motors zeroed", Col.GRN))
                time.sleep(0.8)
            elif ch == "r":
                continue
            elif ch == "q":
                break
    finally:
        node.stop()
        anyset = any(not math.isnan(v) for v in node.ff.values())
        if anyset:
            a = ask(c("\n  Clear FF overrides back to firmware default before exit? [Y/n]: ",
                      Col.YEL)).lower()
            if a[:1] != "n":
                for k in ("coulomb", "viscous", "breakaway"):
                    node.set_ff(k, float("nan"))
                print(c("  overrides cleared", Col.GRN))
        node.stop()
        rclpy.shutdown()
        print("bye")


if __name__ == "__main__":
    main()
