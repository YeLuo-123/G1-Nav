"""Safe real-G1 navigation; hardware output is dry-run by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('g1pilot')
    connect_sdk = LaunchConfiguration('connect_sdk')
    use_rviz = LaunchConfiguration('use_rviz')
    with open(os.path.join(
            share, 'description_files', 'urdf', 'g1_29dof.urdf')) as stream:
        robot_description = stream.read()

    return LaunchDescription([
        DeclareLaunchArgument('connect_sdk', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(
            package='g1pilot', executable='g1_sensor_bridge',
            parameters=[{
                'odom_topic': '/localization/odom',
                'cloud_topic': '/cloud_registered_body',
                'output_frame': 'map',
                'min_height': -0.45,
                'max_height': 0.80,
            }],
            output='screen'),
        Node(
            package='g1pilot', executable='g1_hardware_bridge',
            parameters=[{'connect_sdk': connect_sdk}], output='screen'),
        Node(
            package='g1pilot', executable='dijkstra_planner',
            parameters=[{
                'odom_topic': '/lidar_odometry/pose_fixed',
                'map_topic': '/g1_lidar_slam/map',
            }],
            output='screen'),
        Node(
            package='g1pilot', executable='nav2point',
            parameters=[
                {'vx_limit': 0.20}, {'vy_limit': 0.12},
                {'wz_limit': 0.25}, {'emergency_distance': 0.55}],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(
                share, 'rviz', 'navigation_mapping.rviz')],
            condition=IfCondition(use_rviz), output='screen'),
    ])
