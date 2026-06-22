from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory("open_mower_next"), 'config', 'nav2_params.yaml')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'velocity_smoother',
                       'docking_server',
                       'smoother_server',
                       ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    param_substitutions = {
        'use_sim_time':use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    common_node_params = [configured_params, {'use_sim_time': use_sim_time}]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_node_params,
        remappings=remappings + [('cmd_vel', 'cmd_vel_raw')])
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_node_params,
        remappings=remappings)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_node_params,
        remappings=remappings + [('cmd_vel', 'cmd_vel_raw')])
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_node_params,
        remappings=remappings)
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_node_params,
        remappings=remappings)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_node_params,
        remappings=remappings + [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')])
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=common_node_params,
        remappings=remappings + [('cmd_vel', 'cmd_vel_raw')])
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': lifecycle_nodes}])

    # # Create the launch description and populate
    ld = LaunchDescription([
        stdout_linebuf_envvar,
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        controller_server,
        bt_navigator,
        behavior_server,
        planner_server,
        smoother_server,
        velocity_smoother,
        docking_server,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
    return ld
