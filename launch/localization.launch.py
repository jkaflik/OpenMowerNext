import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def require_env_float(name):
    value = os.getenv(name)
    if value is None:
        raise RuntimeError(f"{name} must be set")
    return float(value)


def wgs84_to_ecef(lat_deg, lon_deg, alt_m=0.0):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    semi_major_axis = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_sq = flattening * (2.0 - flattening)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = semi_major_axis / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)

    x = (radius + alt_m) * cos_lat * math.cos(lon)
    y = (radius + alt_m) * cos_lat * math.sin(lon)
    z = (radius * (1.0 - eccentricity_sq) + alt_m) * sin_lat
    return x, y, z


def generate_launch_description():
    package_path = get_package_share_directory("open_mower_next")

    use_sim_time = LaunchConfiguration("use_sim_time")
    datum_lat = require_env_float("OM_DATUM_LAT")
    datum_lon = require_env_float("OM_DATUM_LONG")
    datum_x, datum_y, datum_z = wgs84_to_ecef(datum_lat, datum_lon)

    fusioncore_params_path = os.path.join(
        package_path, "config", "fusioncore.yaml"
    )

    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[
            fusioncore_params_path,
            {
                "use_sim_time": use_sim_time,
                "reference.use_first_fix": False,
                "reference.x": datum_x,
                "reference.y": datum_y,
                "reference.z": datum_z,
            },
        ],
        remappings=[
            ("/imu/data", "/imu/data_raw"),
            ("/odom/wheels", "/diff_drive_base_controller/odom"),
            ("/gnss/fix", "/gps/fix"),
        ],
    )

    configure_fusioncore = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda action: action is fusioncore_node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    activate_fusioncore = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is fusioncore_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_path, "config", "nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_transform",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            fusioncore_node,
            configure_fusioncore,
            activate_fusioncore,
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="open_mower_next",
                        executable="map_server_node",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "path": os.getenv("OM_MAP_PATH"),
                                "datum": [
                                    float(os.getenv("OM_DATUM_LAT")),
                                    float(os.getenv("OM_DATUM_LONG")),
                                ],
                                "grid.use_gaussian_blur": True,
                            }
                        ],
                        remappings=[
                            ("map_grid", "map_grid"),  # occupancy grid topic
                            ("map", "mowing_map"),  # map topic
                        ],
                    )
                ],
            ),
            Node(
                package="open_mower_next",
                executable="map_recorder",
                name="map_recorder",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="open_mower_next",
                executable="docking_helper",
                name="docking_helper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
