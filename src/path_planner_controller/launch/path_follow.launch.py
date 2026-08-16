from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner_controller',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
        ),
        Node(
            package='path_planner_controller',
            executable='pure_pursuit_controller_node',
            name='pure_pursuit_controller_node',
            output='screen',
        ),
    ])
