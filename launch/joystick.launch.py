from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_joy_node = LaunchConfiguration('enable_joy_node')

    joy_params = os.path.join(get_package_share_directory('open_mower_next'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('joy', '/joy')],
        condition=IfCondition(enable_joy_node),
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('joy', '/joy'), ('cmd_vel', '/cmd_vel_joy')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'enable_joy_node',
            default_value='false',
            description='Start a local joy_node instead of relying on an external /joy publisher'),
        joy_node,
        teleop_node,
    ])
