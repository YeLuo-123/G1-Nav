"""G1Pilot navigation front-end for a dynamics-based locomotion simulator."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='create_map',
            name='dummy_map_publisher',
            parameters=[{'frame_id': 'map'}],
            output='screen',
        ),
        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            output='screen',
        ),
        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            output='screen',
        ),
    ])
