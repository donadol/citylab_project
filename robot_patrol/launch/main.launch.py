from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes
    patrol_with_service_node = Node(
        package='robot_patrol',
        executable='patrol_with_service_executable',
        arguments=[],
        output='screen',
    )

    direction_service = Node(
        package='robot_patrol',
        executable='direction_service',
        arguments=[],
        output='screen',
    )

    return LaunchDescription([
        patrol_with_service_node,
        direction_service
    ])
