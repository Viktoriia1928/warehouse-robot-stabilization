# AMR Stage 4 CV Navigation — Map Integration Subset

This archive contains the **minimal set of project files** responsible for
integrating the warehouse map with the Gazebo world, **without ArUco markers**.

## Included

- Gazebo world file used by the project
- Logical warehouse map (`shared/warehouse_map.yaml`)
- Launch files for Gazebo + CV navigation
- `cv_navigator.py`
- `robot_status_bridge.py`
- Build / run scripts
- GitHub helper files

## Key integration points

1. **Gazebo world**
   - `ros2_ws/src/kolestel_rover_description/worlds/user_saved_layout.sdf`

2. **Logical warehouse map**
   - `shared/warehouse_map.yaml`

3. **Robot pose projection into app coordinates**
   - `web_app/app/ros2_bridge/robot_status_bridge.py`

4. **Navigation logic using the logical map**
   - `ros2_ws/src/kolestel_rover_description/scripts/cv_navigator.py`

## Notes

- This subset intentionally excludes ArUco marker insertion into the world.
- It is intended for GitHub sharing and code review of the map/world integration only.
