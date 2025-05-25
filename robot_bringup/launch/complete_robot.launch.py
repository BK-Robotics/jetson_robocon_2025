import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_controller_pkg',
            executable='main_controller_node',
            name='main_controller_node',
            output='screen',
        ),
        Node(
            package='main_controller_pkg',
            executable='full_calculation_node',
            name='full_calculation_node',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'astra_camera', 'astro_pro_plus.launch.xml'],
            output='screen'
        )
    ])