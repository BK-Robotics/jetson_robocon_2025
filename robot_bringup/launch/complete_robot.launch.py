import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can1',
        description='CAN port for ODrive interface'
    )
    shoot_speed_arg = DeclareLaunchArgument(
        'shoot_speed',
        default_value='20.0',
        description='Shoot speed for ODrive interface'
    )

    return LaunchDescription([
        can_port_arg,
        shoot_speed_arg,
        # Gamepad Node
        Node(
            package='gamepad_interface',
            executable='gamepad_node',
            name='gamepad_node_jetson',
            output='screen',
            remappings=[
                ('/base_cmd', '/base_cmd_jetson'),
                ('/request_mcu', '/request_mcu_jetson'),
                ('/request_odrive', '/request_odrive_jetson'),
                # Add more remaps as needed
            ],
        ),
        # MCU UART Node
        Node(
            package='mcu_interface',
            executable='uart_node',
            name='uart_node_jetson',
            output='screen',
            remappings=[
                ('/base_cmd', '/base_cmd_jetson'),
                ('/imu', '/imu_jetson'),
                ('/rotate_base', '/rotate_base_jetson'),
                ('/request_mcu', '/request_mcu_jetson'),
                ('/push_ball', '/push_ball_jetson'),
                # Add more remaps as needed
            ],
        ),
        # ODrive Interface Node
        Node(
            package='odrive_interface',
            executable='odrive_interface_node',
            name='odrive_interface_node_jetson',
            output='screen',
            parameters=[
                {'can_port': LaunchConfiguration('can_port')},
                {'shoot_speed': LaunchConfiguration('shoot_speed')}
            ],
            remappings=[
                ('/request_odrive', '/request_odrive_jetson'),
                ('/push_ball', '/push_ball_jetson'),
                # Add more remaps as needed
            ],
        )
    ])