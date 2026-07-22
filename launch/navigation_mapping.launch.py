"""SLAM, planning, navigation, robot model, and RViz for MuJoCo G1."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('g1pilot')
    slam_share = get_package_share_directory('slam_toolbox')
    urdf_path = os.path.join(
        package_share, 'description_files', 'urdf', 'g1_29dof.urdf'
    )
    with open(urdf_path) as stream:
        robot_description = stream.read()

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(
                package_share, 'config', 'slam_toolbox.yaml'
            ),
        }.items(),
    )

    return LaunchDescription([
        slam,
        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{'odom_topic': '/lidar_odometry/pose_fixed'}],
            output='screen',
        ),
        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                package_share, 'rviz', 'navigation_mapping.rviz'
            )],
            output='screen',
        ),
    ])
