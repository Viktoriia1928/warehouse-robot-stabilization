cmake_minimum_required(VERSION 3.8)
project(kolestel_rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  worlds
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/pointcloud_to_livox.py
  scripts/video_stabilizer_node.py
  scripts/person_tracker_node.py
  scripts/person_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
