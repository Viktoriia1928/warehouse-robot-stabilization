# Gazebo simulator integration

ROS 2 + Gazebo simulator used to run the full perception stack (stabilization → detection → tracking → person-following) on a virtual robot in a warehouse scene with walking pedestrians. This is the on-robot side of the project; the offline downstream evaluation lives in the rest of this repository.

## What is here

```
gazebo_simulator/
├── README.md                          this file
├── README_original.md                 original package README from the working branch
├── build_ros2.sh                      colcon build helper
├── run_gazebo_tracking.sh             one-shot launcher
├── shared/
│   └── warehouse_map.yaml             logical map (nodes / stations / lanes)
└── ros2_ws/
    └── src/
        └── kolestel_rover_description/
            ├── CMakeLists.txt
            ├── package.xml
            ├── config/
            │   ├── ros_gz_bridge.yaml
            │   ├── tracking.rviz
            │   └── warehouse_map.yaml
            ├── launch/
            │   ├── gazebo.launch.py
            │   └── sim_tracking.launch.py
            ├── meshes/                STL meshes for the rover (chassis, wheels, D435, Livox)
            ├── models/
            │   ├── aruco_marker_0..29 ArUco markers used for landmark experiments
            │   └── walking_person*    Animated pedestrians (6 colours)
            ├── scripts/
            │   ├── odom_to_tf.py            broadcast odom → base_link tf
            │   ├── video_stabilizer_node.py V1–V5 stabilization as a streaming ROS 2 node
            │   ├── person_tracker_node.py   YOLO11n + OC-SORT on the stabilized stream
            │   ├── person_follower_node.py  target-locked visual servoing → /cmd_vel
            │   └── pointcloud_to_livox.py   sensor-conversion utility
            ├── urdf/
            │   └── kolestel_rover.urdf.xacro
            └── worlds/
                └── warehouse_with_people.sdf
```

## Pipeline on the robot

```
camera RGB ──▶ video_stabilizer_node ──▶ person_tracker_node ──▶ person_follower_node ──▶ /cmd_vel
                  (V2 by default,           (YOLO11n +              (longest-track lock,
                   causal Kalman)           OC-SORT)                offset → yaw, area → vx)
```

The ROS 2 stabilizer is the streaming counterpart of `src/stabilization/pipeline.py` from the rest of this repository. Because online operation forbids the offline RTS backward pass, the production V2 in the node runs in causal `filter` mode while the offline notebooks use `rts`; everything else (feature stage, cleanup, smoothing math) is identical.

## What is NOT included

- **AWS RoboMaker warehouse meshes** (`aws_robomaker_warehouse_*`) — these are large public assets that the world file references. They are excluded from this repository to keep its size reasonable.
  Get them from <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world> and copy each `aws_robomaker_warehouse_*` model into `ros2_ws/src/kolestel_rover_description/models/` before launching the world.
- **`yolo11n.pt`** — the detection weights are downloaded on demand by `ultralytics` the first time the tracker runs.
- **ROS 2 build artefacts** (`build/`, `install/`, `log/`) — regenerated locally via `colcon build`.

## Build and run

```
# 1. Workspace build
./build_ros2.sh

# 2. Source the workspace
source ros2_ws/install/setup.bash

# 3. Launch the full sim
./run_gazebo_tracking.sh
# (equivalent to: ros2 launch kolestel_rover_description sim_tracking.launch.py)
```

Requirements: ROS 2 Humble, Gazebo Sim (Garden / Harmonic), `ros_gz` bridge, Nav2 (only used for some auxiliary frames), `ultralytics`, `boxmot`, `cv_bridge`, `vision_msgs`, `tf2_ros`.

## Selecting a stabilization variant at runtime

```
ros2 launch kolestel_rover_description sim_tracking.launch.py video_stabilizer.variant:=V2
ros2 launch kolestel_rover_description sim_tracking.launch.py video_stabilizer.variant:=none
```

`variant:=none` makes the stabilizer publish the raw input — used for the A/B comparison on the robot.
