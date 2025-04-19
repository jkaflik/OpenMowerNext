import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("open_mower_next")

    use_sim_time = LaunchConfiguration("use_sim_time")

    localization_params_path = os.path.join(
        package_path, "config", "robot_localization.yaml"
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
                description="Use simulation (Gazebo) clock if true",
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
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_odom",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", "imu/data_raw"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_map",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu/data", "imu/data_raw"),
                    ("odometry/filtered", "odometry/filtered/map"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    localization_params_path,
                    {
                        "use_sim_time": use_sim_time,
                        "datum": [
                            float(os.getenv("OM_DATUM_LAT")),
                            float(os.getenv("OM_DATUM_LONG")),
                            0.0,
                        ],
                    },
                ],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/map"),
                    ("imu", "gps/orientation"),
                ],
            ),
        ]
    )
