#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"
ACTIVE_WORLD="$ROS_WS/install/kolestel_rover_description/share/kolestel_rover_description/worlds/warehouse_with_people.sdf"
VARIANT="${1:-V2}"

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

if [ ! -f "$ACTIVE_WORLD" ]; then
  echo "World file not found: $ACTIVE_WORLD" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

echo "[run_gazebo_tracking] Using world: $ACTIVE_WORLD"
echo "[run_gazebo_tracking] Stabilization variant: $VARIANT"
echo "[run_gazebo_tracking] Tip: 'none' = passthrough, 'V2' = LK+Kalman (production), 'V5' = robust LK+Kalman"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

export AMR_WAREHOUSE_MAP_PATH="$ROOT_DIR/shared/warehouse_map.yaml"
export AMR_GAZEBO_WORLD_PATH="$ACTIVE_WORLD"

ros2 launch kolestel_rover_description sim_tracking.launch.py variant:="$VARIANT"
