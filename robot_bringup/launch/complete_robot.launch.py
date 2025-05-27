import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First, execute the astro camera launch file.
        ExecuteProcess(
            cmd=['ros2', 'launch', 'astra_camera', 'astro_pro_plus.launch.xml'],
            output='screen',
        ),
        # Delay 2 seconds and then start all the nodes.
        TimerAction(
            period=2.0,
            actions=[
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
                Node(
                    package='main_controller_pkg',
                    executable='tf_publish_node',
                    name='tf_publish_node',
                    output='screen',
                ),
                Node(
                    package='main_controller_pkg',
                    executable='plane_calculation_node',
                    name='plane_calculation_node',
                    output='screen',
                ),
            ]
        ),
        # Optionally, uncomment to add further process launch commands.
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', '/home/bkrobotics/jetson_robocon_2025/jetson_robocon_2025/robot_bringup/rviz/point_cloud.rviz'],
        #     output='screen'
        # )
    ])