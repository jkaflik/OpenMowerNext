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
from launch_ros.parameter_descriptions import ParameterValue
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
    gnss_base_noise_xy = LaunchConfiguration("gnss_base_noise_xy")
    gnss_track_heading_min_speed = LaunchConfiguration("gnss_track_heading_min_speed")
    gnss_track_heading_min_dist = LaunchConfiguration("gnss_track_heading_min_dist")
    gnss_heading_observable_distance = LaunchConfiguration("gnss_heading_observable_distance")
    gnss_use_gps_fix = LaunchConfiguration("gnss_use_gps_fix")
    gnss_fix_topic = LaunchConfiguration("gnss_fix_topic")
    init_stationary_window = LaunchConfiguration("init_stationary_window")
    init_wait_for_all_sensors = LaunchConfiguration("init_wait_for_all_sensors")
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
                "gnss.base_noise_xy": ParameterValue(gnss_base_noise_xy, value_type=float),
                "gnss.track_heading_min_speed": ParameterValue(
                    gnss_track_heading_min_speed, value_type=float
                ),
                "gnss.track_heading_min_dist": ParameterValue(
                    gnss_track_heading_min_dist, value_type=float
                ),
                "gnss.heading_observable_distance": ParameterValue(
                    gnss_heading_observable_distance, value_type=float
                ),
                "gnss.use_gps_fix": ParameterValue(gnss_use_gps_fix, value_type=bool),
                "init.stationary_window": ParameterValue(init_stationary_window, value_type=float),
                "init.wait_for_all_sensors": ParameterValue(
                    init_wait_for_all_sensors, value_type=bool
                ),
            },
        ],
        remappings=[
            ("/imu/data", "/imu/data_raw"),
            ("/odom/wheels", "/diff_drive_base_controller/odom"),
            ("/gnss/fix", gnss_fix_topic),
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
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "gnss_base_noise_xy",
                default_value="0.5",
                description="GNSS horizontal base noise for FusionCore",
            ),
            DeclareLaunchArgument(
                "gnss_track_heading_min_speed",
                default_value="0.2",
                description="Minimum speed to count GNSS track motion for heading observability",
            ),
            DeclareLaunchArgument(
                "gnss_track_heading_min_dist",
                default_value="5.0",
                description="GNSS track baseline before fusing track heading",
            ),
            DeclareLaunchArgument(
                "gnss_heading_observable_distance",
                default_value="5.0",
                description="Valid GNSS track distance before enabling GNSS lever-arm correction",
            ),
            DeclareLaunchArgument(
                "gnss_use_gps_fix",
                default_value="false",
                description="Subscribe to gps_msgs/GPSFix instead of sensor_msgs/NavSatFix",
            ),
            DeclareLaunchArgument(
                "gnss_fix_topic",
                default_value="/gps/fix",
                description="GNSS fix topic remapped into FusionCore /gnss/fix",
            ),
            DeclareLaunchArgument(
                "init_stationary_window",
                default_value="2.0",
                description="Seconds of stationary IMU data to collect before FusionCore init",
            ),
            DeclareLaunchArgument(
                "init_wait_for_all_sensors",
                default_value="true",
                description="Wait for every configured FusionCore sensor before initialization",
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
