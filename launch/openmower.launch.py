import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def controller_parameters_file(share_directory):
    controller_path = os.path.join(share_directory, 'config', 'controllers.yaml')
    hardware_path = os.path.join(share_directory, 'config', 'hardware', 'yardforce500.yaml')
    with open(controller_path, 'r', encoding='utf-8') as stream:
        controllers = yaml.safe_load(stream)
    with open(hardware_path, 'r', encoding='utf-8') as stream:
        hardware = yaml.safe_load(stream)
    wheel_offset_y = float(hardware['wheel']['offset'][1])
    controllers.setdefault('diff_drive_base_controller', {}).setdefault('ros__parameters', {})[
        'wheel_separation'
    ] = 2.0 * abs(wheel_offset_y)

    params_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='openmower_controllers_', suffix='.yaml', delete=False
    )
    with params_file:
        yaml.safe_dump(controllers, params_file, sort_keys=False)
    return params_file.name


def generate_launch_description():
    package_name = 'open_mower_next'

    share_directory = get_package_share_directory(package_name)
    enable_foxglove = LaunchConfiguration('enable_foxglove')
    foxglove_address = LaunchConfiguration('foxglove_address')
    foxglove_port = LaunchConfiguration('foxglove_port')
    enable_joy_node = LaunchConfiguration('enable_joy_node')
    enable_navigation_readiness = LaunchConfiguration('enable_navigation_readiness')

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
        )]), launch_arguments={
            'use_sim_time': 'false',
            'enable_joy_node': enable_joy_node,
        }.items()
    )

    twist_mux_params = os.path.join(share_directory, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    controller_params_file = controller_parameters_file(share_directory)

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

    navigation_readiness = Node(
        package='open_mower_next',
        executable='navigation_readiness_node',
        output='screen',
        parameters=[{'use_sim_time': False, 'timeout_seconds': 120.0}],
        condition=IfCondition(enable_navigation_readiness),
    )

    def make_nav2(condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/nav2.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
            condition=condition,
        )

    nav2 = make_nav2()
    nav2_without_readiness = make_nav2(UnlessCondition(enable_navigation_readiness))

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            )
        ),
        launch_arguments={
            'address': foxglove_address,
            'port': foxglove_port,
            'include_hidden': 'true',
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(enable_foxglove),
    )

    def start_nav2_or_shutdown(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            return [nav2]
        return [EmitEvent(event=Shutdown(reason='Navigation prerequisites timed out'))]

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument('enable_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('enable_joy_node', default_value='false'),
        DeclareLaunchArgument('enable_navigation_readiness', default_value='true'),

        node_robot_state_publisher,
        joystick,
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
                target_action=load_diff_controller,
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
                'gnss_use_gps_fix': 'true',
                'gnss_fix_topic': '/gps/fix_extended',
            }.items(),
        ),

        navigation_readiness,
        nav2_without_readiness,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=navigation_readiness,
                on_exit=start_nav2_or_shutdown,
            ),
            condition=IfCondition(enable_navigation_readiness),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [share_directory, '/launch/micro_ros_agent.launch.py']),
        ),

        foxglove_bridge,
    ])
