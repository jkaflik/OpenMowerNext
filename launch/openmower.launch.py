import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = 'open_mower_next'

    share_directory = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_directory, 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': '1',
        'use_sim_time': '0'
    }).toxml()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            share_directory, 'launch', 'joystick.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    twist_mux_params = os.path.join(share_directory, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    controller_params_file = os.path.join(share_directory, 'config', 'controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_config},
                    controller_params_file]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    load_mower_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mower_controller'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        node_robot_state_publisher,
        twist_mux,
        controller_manager,

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_mower_controller],
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/gps.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/localization.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/nav2.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [share_directory, '/launch/micro_ros_agent.launch.py']),
        ),
    ])
