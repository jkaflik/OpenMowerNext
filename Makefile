REMOTE_HOST ?= omdev.local
REMOTE_USER ?= openmower
ROS_LOG_DIR = log/
SHELL := /bin/bash

all: custom-deps deps build

.PHONY: deps build

deps:
	rosdep install --from-paths . src/lib --ignore-src -i -y -r

custom-deps:
	sh utils/install-custom-deps.sh

build-libs:
	colcon build --base-paths "src/lib/*" --cmake-args -DBUILD_TESTING=OFF

build:
	colcon build --symlink-install

build-release:
	colcon build --base-paths "src/lib/*" --cmake-args -DCMAKE_BUILD_TYPE=Release
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

sim:
	bash -lc 'if [ -z "$${WEBOTS_HOME}" ] && [ -d "$${HOME}/.ros/webotsR2025a/webots" ]; then export WEBOTS_HOME="$${HOME}/.ros/webotsR2025a/webots"; fi && if [ -z "$${DISPLAY}" ] && [ -z "$${WEBOTS_OFFSCREEN}" ]; then export WEBOTS_OFFSCREEN=1; fi && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash && set -a && source .devcontainer/default.env && set +a && ros2 launch open_mower_next sim.launch.py'

run:
	ros2 launch launch/openmower.launch.py

dev:
	cd .devcontainer && docker-compose up -d

run-foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

rsp:
	ros2 launch launch/rsp.launch.py

remote-devices:
	bash .devcontainer/scripts/remote_devices.sh $(REMOTE_HOST) $(REMOTE_USER)
