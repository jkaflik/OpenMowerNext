import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nodes = [
        Node(
            package='ublox_f9p',
            executable='ublox_f9p',
            output='both',
            parameters=[os.path.join(get_package_share_directory("open_mower_next"), 'config', 'gps.yaml')],
        ),
    ]

    # check if OM_NTRIP_ENABLED is set to true
    if os.getenv('OM_NTRIP_ENABLED') == 'true':
        nodes.append(
            Node(
                name='ntrip_client_node',
                package='ntrip_client',
                executable='ntrip_ros.py',
                parameters=[{
                    'host': os.getenv('OM_NTRIP_HOSTNAME', ''),
                    'port': int(os.getenv('OM_NTRIP_PORT', '2101')),
                    'mountpoint': os.getenv('OM_NTRIP_ENDPOINT', ''),
                    'authenticate': os.getenv('OM_NTRIP_AUTHENTICATE', 'false') == 'true',
                    'username': os.getenv('OM_NTRIP_USER', ''),
                    'password': os.getenv('OM_NTRIP_PASSWORD', ''),
                    'rtcm_message_package': 'rtcm_msgs',
                }],
                remappings=[('fix', '/gps/fix')],
            ),
        )

    return LaunchDescription(nodes)
