from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes
    test_service = Node(
        package='robot_patrol',
        executable='test_service',
        arguments=[],
        output='screen',
    )

    return LaunchDescription([
        test_service
    ])