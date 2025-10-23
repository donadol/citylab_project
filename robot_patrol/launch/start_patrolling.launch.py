import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get Package Description and Directory #
    package_description = "robot_patrol"
    package_directory = get_package_share_directory(package_description)

    # Nodes
    patrol_node = Node(
        package='robot_patrol',
        executable='patrol_executable',
        arguments=[],
        output='screen',
    )

    rviz_config_file = os.path.join(package_directory, "config", "display.rviz")
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output="screen",
        emulate_tty=True,
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        patrol_node,
        rviz2_node
    ])