#!/usr/bin/env python3
"""Focused capture for the manual-collision -> return-to-dock bug (#3).

Run this, then drive the robot into an obstacle while in MANUAL_MOWING.
It records a timestamped JSONL timeline and prints notable events live, so
we can see exactly which signal flips the robot out of manual and what
(if anything) commands it to the dock.

What it watches:
  - /behavior_tree_node/high_level_status (HighLevelStatus): state changes
  - /behavior_tree_log (nav2_msgs/BehaviorTreeLog): EVERY BT node transition
    (this is the key one -- it names the node that fires the home/dock path)
  - /hardware_bridge/drive_telemetry (Float32MultiArray): slip flags incl IMPACT
  - /hardware_bridge/emergency (mowgli_interfaces/Emergency): latch changes
  - /battery_state: charging-status changes
  - /cmd_vel, /cmd_vel_teleop: motion presence

Usage (inside the container):
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash
  python3 /ros2_ws/scripts/drive_collision_trace.py --output-dir /ros2_ws/maps
Ctrl-C to stop; a summary timeline is printed and written.
"""
import argparse
import datetime as dt
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mowgli_interfaces.msg import HighLevelStatus, Emergency
from nav2_msgs.msg import BehaviorTreeLog
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, TwistStamped

STATE_NAMES = {0: "NULL", 1: "IDLE", 2: "AUTONOMOUS", 3: "RECORDING", 4: "MANUAL_MOWING"}
SLIP_FLAGS = [
    (1 << 0, "YAW"),
    (1 << 1, "STALL"),
    (1 << 2, "IMPACT"),
    (1 << 3, "BOG"),
    (1 << 4, "BLADE_BOG"),
    (1 << 5, "JAM"),
]
# BT node names that indicate a home/dock/undock/recovery path being taken.
HOME_HINTS = ("home", "dock", "navigate", "return", "charg", "emergency",
              "undock", "idle", "gohome", "transit")


def decode_flags(bits):
    bits = int(bits)
    return [name for mask, name in SLIP_FLAGS if bits & mask]


class CollisionTrace(Node):
    def __init__(self, out_path):
        super().__init__("drive_collision_trace")
        self.out = open(out_path, "w", buffering=1)
        self.out_path = out_path
        self.last_state = None
        self.last_flags = 0
        self.last_emergency = None
        self.last_charging = None
        self.events = []  # notable events for the summary

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(HighLevelStatus,
                                 "/behavior_tree_node/high_level_status",
                                 self.on_status, reliable)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log",
                                 self.on_bt_log, reliable)
        self.create_subscription(Float32MultiArray,
                                 "/hardware_bridge/drive_telemetry",
                                 self.on_telem, sensor)
        self.create_subscription(Emergency, "/hardware_bridge/emergency",
                                 self.on_emergency, reliable)
        self.create_subscription(BatteryState, "/battery_state",
                                 self.on_battery, sensor)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, sensor)
        self.create_subscription(TwistStamped, "/cmd_vel_teleop",
                                 self.on_cmd_teleop, sensor)

        self._meta()
        print(f"[trace] writing {out_path}", flush=True)
        print("[trace] drive into the obstacle in MANUAL_MOWING now. Ctrl-C to stop.\n",
              flush=True)

    # --- helpers ---
    def _now(self):
        return dt.datetime.now(dt.timezone.utc).isoformat()

    def _write(self, rec):
        rec["t"] = self._now()
        # default=str so ROS Time/builtin types (e.g. Emergency stamps) serialize.
        self.out.write(json.dumps(rec, default=str) + "\n")

    def _notable(self, msg):
        line = f"{self._now()}  {msg}"
        self.events.append(line)
        print(line, flush=True)

    def _meta(self):
        self._write({"kind": "meta", "note": "drive_collision_trace #3"})

    # --- callbacks ---
    def on_status(self, m):
        st = int(getattr(m, "state", -1))
        self._write({"kind": "high_level_status", "state": st,
                     "state_name": STATE_NAMES.get(st, str(st))})
        if st != self.last_state:
            self._notable(f"STATE  {STATE_NAMES.get(self.last_state, self.last_state)}"
                          f" -> {STATE_NAMES.get(st, st)}")
            self.last_state = st

    def on_bt_log(self, m):
        for ev in m.event_log:
            rec = {"kind": "bt_event", "node": ev.node_name,
                   "prev": ev.previous_status, "cur": ev.current_status}
            self._write(rec)
            name = ev.node_name.lower()
            # Flag transitions into RUNNING/SUCCESS for home/dock-ish nodes.
            if any(h in name for h in HOME_HINTS) and ev.current_status in (
                    "RUNNING", "SUCCESS", "IDLE"):
                self._notable(f"BT     {ev.node_name}: {ev.previous_status}"
                              f" -> {ev.current_status}")

    def on_telem(self, m):
        if len(m.data) < 13:
            return
        flags = int(m.data[12])
        self._write({"kind": "telem", "slip_flags": flags,
                     "flags": decode_flags(flags)})
        if flags != self.last_flags:
            self._notable(f"SLIP   {decode_flags(self.last_flags)}"
                          f" -> {decode_flags(flags)}")
            self.last_flags = flags

    def on_emergency(self, m):
        # Emergency fields vary; capture all bool-ish attributes generically.
        fields = {k: getattr(m, k) for k in m.get_fields_and_field_types()}
        self._write({"kind": "emergency", "fields": fields})
        # Compare only meaningful fields (the stamp changes every message).
        meaningful = {k: v for k, v in fields.items() if k != "stamp"}
        snap = json.dumps(meaningful, default=str, sort_keys=True)
        if snap != self.last_emergency:
            self._notable(f"EMERG  {snap}")
            self.last_emergency = snap

    def on_battery(self, m):
        charging = int(getattr(m, "power_supply_status", 0))
        self._write({"kind": "battery", "power_supply_status": charging,
                     "current": float(getattr(m, "current", 0.0))})
        if charging != self.last_charging:
            self._notable(f"BATT   power_supply_status -> {charging}")
            self.last_charging = charging

    def on_cmd_vel(self, m):
        self._write({"kind": "cmd_vel", "vx": round(m.linear.x, 3),
                     "wz": round(m.angular.z, 3)})

    def on_cmd_teleop(self, m):
        self._write({"kind": "cmd_vel_teleop", "vx": round(m.twist.linear.x, 3),
                     "wz": round(m.twist.angular.z, 3)})

    def finish(self):
        print("\n===== timeline (notable events) =====", flush=True)
        for line in self.events:
            print(line, flush=True)
        self._write({"kind": "summary", "notable_events": self.events})
        self.out.close()
        print(f"\n[trace] full JSONL: {self.out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", default="/ros2_ws/maps")
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_path = os.path.join(args.output_dir, f"collision_trace_{ts}.jsonl")

    rclpy.init()
    node = CollisionTrace(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
