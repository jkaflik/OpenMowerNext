import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher, WebotsLauncher


def shutdown_on_driver_failure(event, context):
    if context.is_shutdown or event.returncode == 0:
        return []
    return [EmitEvent(event=Shutdown(reason="Webots driver exited unexpectedly"))]


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    package_name = "open_mower_next"
    share_directory = get_package_share_directory(package_name)

    world = LaunchConfiguration("world").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    webots_stream = LaunchConfiguration("webots_stream").perform(context).lower() in [
        "true",
        "1",
        "yes",
    ]
    webots_port = LaunchConfiguration("webots_port").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")

    xacro_file = os.path.join(share_directory, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "use_ros2_control": "0",
            "sim_mode": "true",
        },
    ).toxml()

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config, "use_sim_time": use_sim_time}],
    )

    twist_mux_params = os.path.join(share_directory, "config", "twist_mux.yaml")
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/diff_drive_base_controller/cmd_vel")],
    )

    webots = WebotsLauncher(
        world=os.path.join(share_directory, "worlds", world),
        mode=mode,
        gui=gui,
        stream=webots_stream,
        port=webots_port,
    )
    webots_supervisor = Ros2SupervisorLauncher(respawn=False)

    controller_params_file = os.path.join(share_directory, "config", "controllers.yaml")
    webots_robot_description = os.path.join(share_directory, "resource", "openmower_webots.urdf")
    webots_driver = WebotsController(
        robot_name="openmower",
        parameters=[
            {
                "robot_description": webots_robot_description,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": False,
            },
            controller_params_file,
        ],
        respawn=False,
    )

    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    load_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_drive_base_controller"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    load_mower_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["mower_controller"] + controller_manager_timeout,
        parameters=[{"use_sim_time": use_sim_time}],
    )
    controller_spawners = [load_joint_state_controller, load_diff_controller, load_mower_controller]

    wait_for_webots_driver = WaitForControllerConnection(
        target_driver=webots_driver,
        nodes_to_start=controller_spawners,
    )

    # Simulation helper node publishes the hardware-facing power topics.
    sim_node = Node(
        package="open_mower_next",
        executable="sim_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "docking_station_frame": "map",
                "charging_port_frame": "charging_port",
                "docking_station_contact_x": 1.82,
                "docking_station_contact_y": 1.5,
                "docking_station_contact_z": 0.06,
                "docking_station_contact_yaw": 0.0,
            }
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_directory, "launch", "localization.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_directory, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": os.path.join(share_directory, "config", "nav2_params.yaml"),
        }.items(),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml")
        ),
        launch_arguments={
            "address": foxglove_address,
            "port": foxglove_port,
            "include_hidden": "true",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(enable_foxglove),
    )

    return [
        webots,
        webots_supervisor,
        node_robot_state_publisher,
        twist_mux,
        webots_driver,
        wait_for_webots_driver,
        sim_node,
        localization,
        nav2,
        foxglove_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots_driver,
                on_exit=shutdown_on_driver_failure,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="openmower.wbt"),
            DeclareLaunchArgument("mode", default_value="realtime"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("webots_stream", default_value="true"),
            DeclareLaunchArgument("webots_port", default_value="1234"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_foxglove", default_value="false"),
            DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            OpaqueFunction(function=launch_setup),
        ]
    )
