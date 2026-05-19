# amr_stage5_tracking — стабилизация + трекинг + следование за человеком

Минимальный ROS2 пакет, собранный поверх `kolestel_rover_description` из
`amr_stage4_cv_nav`, но без ArUco / Nav2 / линейного следования. Здесь:

- **Gazebo мир** `worlds/warehouse_with_people.sdf` со складом и **5 ходящими
  людьми** (`<actor>` с walk-анимацией; каждый человек получает свой ID в OC-SORT).
- **video_stabilizer_node.py** — 5 вариантов стабилизации из диссертации
  (V1 LK+MA, V2 LK+Kalman, V3 ORB+MA, V4 robust LK+MA, V5 robust LK+Kalman),
  выбирается ROS-параметром `variant`. Kalman в ноде каузальный (forward-only).
- **person_tracker_node.py** — YOLO11n + OC-SORT поверх стабилизированного
  потока. Публикует аннотированный кадр и `vision_msgs/Detection2DArray`.
- **person_follower_node.py** — лочится на самый долгоживущий ID и едет к нему
  (cmd_vel формируется из горизонтального смещения bbox + размера bbox как
  proxy расстояния).

## Структура

```
amr_stage5_tracking/
├── build_ros2.sh
├── run_gazebo_tracking.sh
├── shared/warehouse_map.yaml
└── ros2_ws/src/kolestel_rover_description/
    ├── worlds/warehouse_with_people.sdf
    ├── launch/
    │   ├── gazebo.launch.py
    │   └── sim_tracking.launch.py
    ├── scripts/
    │   ├── video_stabilizer_node.py
    │   ├── person_tracker_node.py
    │   ├── person_follower_node.py
    │   ├── odom_to_tf.py
    │   └── pointcloud_to_livox.py
    ├── urdf/kolestel_rover.urdf.xacro
    ├── config/{ros_gz_bridge.yaml,warehouse_map.yaml}
    ├── models/aws_robomaker_warehouse_*/
    └── meshes/
```

## Зависимости

```bash
# системные
sudo apt install ros-jazzy-vision-msgs ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
                 ros-jazzy-realsense2-description

# python
pip install ultralytics boxmot opencv-python
```

При первом запуске:
- `ultralytics` сам скачает `yolo11n.pt` в `~/.cache/`.
- Gazebo Harmonic при первом запуске скачает walk.dae с Fuel
  (`https://fuel.gazebosim.org/...`) — нужен интернет.

## Запуск

```bash
./build_ros2.sh
./run_gazebo_tracking.sh        # вариант стабилизации по умолчанию = V2
./run_gazebo_tracking.sh V5     # robust LK + Kalman
./run_gazebo_tracking.sh none   # passthrough (без стабилизации, для A/B)
```

После запуска проверить топики:
```bash
ros2 topic hz /camera/image_stabilized
ros2 topic echo /tracking/target_id
ros2 run rqt_image_view rqt_image_view /tracking/image
```

## Топики

| Топик | Тип | Источник |
| --- | --- | --- |
| `/camera/image` | sensor_msgs/Image | Gazebo (RealSense D435 RGB) |
| `/camera/image_stabilized` | sensor_msgs/Image | video_stabilizer_node |
| `/tracking/image` | sensor_msgs/Image | person_tracker_node (аннотированный) |
| `/tracking/persons` | vision_msgs/Detection2DArray | person_tracker_node |
| `/tracking/target_id` | std_msgs/Int32 | person_follower_node |
| `/cmd_vel` | geometry_msgs/Twist | person_follower_node → Gazebo |

## Параметры

- `variant` (sim_tracking.launch.py): `V1` / `V2` / `V3` / `V4` / `V5` / `none`
- В follower: `linear_speed`, `angular_gain`, `stop_bbox_height_frac`,
  `min_bbox_height_frac`, `lost_grace_frames`, `relock_after_lost`.
