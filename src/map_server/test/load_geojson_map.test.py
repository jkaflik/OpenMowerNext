import os
import sys

from threading import Event
from threading import Thread
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
from launch_testing.io_handler import ActiveIoHandler
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from open_mower_next.msg import Map
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    test_map = os.path.join(path_to_test, 'test_map.geojson')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            # executable=sys.executable,
            package='open_mower_next',
            executable='map_server_node',
            name='map_server',
            output='screen',
            parameters=[{
                'type': 'geojson',
                'path': test_map,
            }],
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestMapServer(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_map_server')

    def tearDown(self):
        self.node.destroy_node()

    def test_map(self):
        self.node.get_logger().info('Waiting for map...')

        actual_map = Map()
        event = Event()

        self.node.create_subscription(Map, '/map', lambda msg:
            [actual_map.__setattr__(key, value) for key, value in msg.__dict__.items()]
            or event.set()
        , 1)

        assert event.wait(timeout=10), 'Map not received'
