#!/usr/bin/env bash

set -e

SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd "$SCRIPT_PATH/.."

mkdir -p src/lib
vcs import src/lib --force --shallow < custom_deps.yaml

if [ -d src/lib/webots_ros2 ]; then
  for package in \
    webots_ros2 \
    webots_ros2_control \
    webots_ros2_crazyflie \
    webots_ros2_epuck \
    webots_ros2_husarion \
    webots_ros2_importer \
    webots_ros2_mavic \
    webots_ros2_msgs \
    webots_ros2_tesla \
    webots_ros2_tests \
    webots_ros2_tiago \
    webots_ros2_turtlebot \
    webots_ros2_universal_robot
  do
    touch "src/lib/webots_ros2/${package}/COLCON_IGNORE"
    touch "src/lib/webots_ros2/${package}/AMENT_IGNORE"
  done
fi

rosdep install --from-paths src/lib --ignore-src --rosdistro $ROS_DISTRO -y
