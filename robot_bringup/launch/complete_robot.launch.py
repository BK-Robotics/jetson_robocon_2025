from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_controller_pkg',
            executable='main_controller_node',
            name='main_controller_node',
            output='screen',
            # parameters=[{'param_name': 'param_value'}],
            # remappings=[('/old/topic', '/new/topic')]
        ),
        Node(
            package='main_controller_pkg',
            executable='full_calculation_node',
            name='full_calculation_node',
            output='screen',
            # parameters=[{'param_name': 'param_value'}],
            # remappings=[('/old/topic', '/new/topic')]
        )
        # Node(
        #     package='imu_pkg',
        #     executable='imu_node',
        #     name='imu_node',
        #     output='screen'
        # ),
        # Node(
        #     package='control_pkg',
        #     executable='control_node',
        #     name='control_node',
        #     output='screen'
        # ),
        # Node(
        #     package='request_calculation_pkg',
        #     executable='request_calculation_node',
        #     name='request_calculation_node',
        #     output='screen'
        # ),
        # Node(
        #     package='rotate_base_pkg',
        #     executable='rotate_base_node',
        #     name='rotate_base_node',
        #     output='screen'
        # )
    ])