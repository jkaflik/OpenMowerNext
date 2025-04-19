from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory("open_mower_next"), 'config', 'nav2_params.yaml')

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
        ('battery_state', 'power'),
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

    create_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
        output='screen')

    load_composable_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')],),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')]),
            ComposableNode(
                package='opennav_docking',
                plugin='opennav_docking::DockingServer',
                name='docking_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_raw')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # # Create the launch description and populate
    ld = LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory("open_mower_next"), 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        create_container,
        load_composable_nodes
    ])
    return ld