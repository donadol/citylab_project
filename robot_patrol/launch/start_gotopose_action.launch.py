from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes
    go_to_pose_action = Node(
        package='robot_patrol',
        executable='go_to_pose_action',
        arguments=[],
        output='screen',
    )

    return LaunchDescription([
        go_to_pose_action
    ])