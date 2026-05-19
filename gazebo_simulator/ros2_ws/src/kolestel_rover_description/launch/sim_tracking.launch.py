"""Full simulation: Gazebo world with walking people +
video stabilizer + YOLO/OC-SORT tracker + visual follower + optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py'))
    )

    variant = LaunchConfiguration('variant')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = os.path.join(pkg, 'config', 'tracking.rviz')

    stabilizer = Node(
        package='kolestel_rover_description',
        executable='video_stabilizer_node.py',
        name='video_stabilizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'variant': variant,
            'image_topic': '/camera/image',
            'output_topic': '/camera/image_stabilized',
        }],
    )

    tracker = Node(
        package='kolestel_rover_description',
        executable='person_tracker_node.py',
        name='person_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_stabilized',
            'annotated_topic': '/tracking/image',
            'detections_topic': '/tracking/persons',
            'yolo_weights': 'yolo11n.pt',
            'conf': 0.25,
            'iou': 0.45,
            'imgsz': 640,
        }],
    )

    follower = Node(
        package='kolestel_rover_description',
        executable='person_follower_node.py',
        name='person_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'detections_topic': '/tracking/persons',
            'camera_info_topic': '/camera/camera_info',
            'cmd_vel_topic': '/cmd_vel',
            'linear_speed': 0.55,
            'far_boost': 1.8,
            'angular_gain': 0.0035,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='V2',
            description='Stabilization variant: V1, V2, V3, V4, V5, or none',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with the tracking config',
        ),
        gazebo_launch,
        TimerAction(period=12.0, actions=[stabilizer]),
        TimerAction(period=14.0, actions=[tracker]),
        TimerAction(period=16.0, actions=[follower]),
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
