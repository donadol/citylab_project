from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes
    direction_service = Node(
        package='robot_patrol',
        executable='direction_service',
        arguments=[],
        output='screen',
    )

    return LaunchDescription([
        direction_service
    ])