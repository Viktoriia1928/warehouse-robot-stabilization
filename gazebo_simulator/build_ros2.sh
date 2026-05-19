#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"

deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "[build_ros2] Cleaning colcon caches..."
rm -rf "$ROS_WS/build" "$ROS_WS/install" "$ROS_WS/log"

cd "$ROS_WS"
colcon build --packages-select kolestel_rover_description

echo "[build_ros2] Done."
