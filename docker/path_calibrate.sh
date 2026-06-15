#!/usr/bin/env bash
# Launch the path-following calibration TUI inside the mowgli-ros2 container,
# with the ROS environment sourced. Run from the host:
#     ./docker/path_calibrate.sh
set -euo pipefail

CONTAINER="${MOWGLI_ROS2_CONTAINER:-mowgli-ros2}"
SCRIPT="${PATH_CALIBRATE_PATH:-/ros2_ws/maps/path_calibrate.py}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "error: container '$CONTAINER' is not running (start the stack first)." >&2
  exit 1
fi

exec docker exec -it "$CONTAINER" bash -lc \
  "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && exec python3 '$SCRIPT'"
