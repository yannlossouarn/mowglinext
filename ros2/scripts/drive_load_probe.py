#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""drive_load_probe — observe the firmware drive telemetry to calibrate the
per-wheel load thresholds (and see the discrepancy flags) before baking them in.

Subscribes /hardware_bridge/drive_telemetry (std_msgs/Float32MultiArray, ~25 Hz)
published by hardware_bridge from the firmware drive-telem packet. Layout:
  [0] l_target  [1] r_target  [2] l_actual  [3] r_actual   (m/s)
  [4] l_pwm     [5] r_pwm                                   (signed PWM)
  [6] wheel_yaw [7] imu_yaw    [8] yaw_residual             (rad/s)
  [9] accel_peak_g
  [10] left_load [11] right_load                            (0-255, uncalibrated)
  [12] slip_flags

It prints a live one-liner and auto-buckets the per-wheel LOAD by motion regime
so you can read off the bands to set the firmware thresholds:
  idle      — no command            -> baseline / electrical noise floor
  driving   — commanded & moving     -> normal load envelope
  stalled   — commanded & not moving -> jam / obstruction (HIGH load expected)
For a free-slip sample, lift a wheel (or spin on something slick) while
commanding forward — it shows up under "driving" with an unusually LOW load.

On Ctrl-C it prints a per-regime, per-wheel load summary (p10 / median / p90).
Hand me those numbers (or this printout) and I'll set the thresholds.

Run in-container:
  docker exec -it mowgli-ros2 bash -lc \\
    'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \\
     python3 /ros2_ws/scripts/drive_load_probe.py'
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

TOPIC = "/hardware_bridge/drive_telemetry"
FLAG_NAMES = [(1 << 0, "YAW"), (1 << 1, "STALL"), (1 << 2, "IMPACT"),
              (1 << 3, "BOG"), (1 << 4, "BLADE_BOG")]
MOVING_MPS = 0.03   # |actual| above this = the wheel is turning
CMD_MPS = 0.05      # |target| above this = a real command


def decode_flags(bits):
    bits = int(bits)
    on = [name for mask, name in FLAG_NAMES if bits & mask]
    return ",".join(on) if on else "-"


def regime(cmd, act):
    if cmd < CMD_MPS:
        return "idle"
    return "driving" if act >= MOVING_MPS else "stalled"


class LoadProbe(Node):
    def __init__(self):
        super().__init__("drive_load_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32MultiArray, TOPIC, self._cb, qos)
        # buckets[regime][wheel] -> list of load samples
        self.buckets = {r: {"L": [], "R": []} for r in ("idle", "driving", "stalled")}
        self.n = 0
        self.last_print = 0.0
        self.flag_events = 0
        self.get_logger().info(f"listening on {TOPIC} — drive the robot through "
                               "idle / forward / a deliberate stall, then Ctrl-C")

    def _cb(self, msg):
        d = msg.data
        if len(d) < 13:
            return
        self.n += 1
        l_cmd, r_cmd, l_act, r_act = abs(d[0]), abs(d[1]), abs(d[2]), abs(d[3])
        l_load, r_load = d[10], d[11]
        flags = int(d[12])
        self.buckets[regime(l_cmd, l_act)]["L"].append(l_load)
        self.buckets[regime(r_cmd, r_act)]["R"].append(r_load)

        now = time.time()
        if flags or now - self.last_print > 0.5:
            self.last_print = now
            if flags:
                self.flag_events += 1
            print(f"\rload L/R={l_load:3.0f}/{r_load:3.0f}  "
                  f"v L/R={d[2]:+.2f}/{d[3]:+.2f}  pwm={d[4]:+.0f}/{d[5]:+.0f}  "
                  f"resid={d[8]:+.2f} peak={d[9]:.2f}g  flags=[{decode_flags(flags)}]    ",
                  end="", flush=True)

    def summary(self):
        print("\n\n===== per-wheel LOAD by regime (0-255) =====")
        print(f"{'regime':<10}{'wheel':<6}{'n':>7}{'p10':>7}{'median':>8}{'p90':>7}{'max':>6}")
        for r in ("idle", "driving", "stalled"):
            for w in ("L", "R"):
                s = sorted(self.buckets[r][w])
                if not s:
                    print(f"{r:<10}{w:<6}{0:>7}{'—':>7}{'—':>8}{'—':>7}{'—':>6}")
                    continue
                p10 = s[int(len(s) * 0.1)]
                p90 = s[int(len(s) * 0.9)]
                print(f"{r:<10}{w:<6}{len(s):>7}{p10:>7.0f}"
                      f"{statistics.median(s):>8.0f}{p90:>7.0f}{max(s):>6.0f}")
        print(f"\nsamples={self.n}  flag-bearing prints={self.flag_events}")
        print("Set 'high load' between the driving-p90 and stalled-median; "
              "'low/free' below the driving-p10.")


def main():
    rclpy.init()
    node = LoadProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
