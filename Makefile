REMOTE_HOST ?= omdev.local
REMOTE_USER ?= openmower
ROS_DISTRO ?= jazzy
ROS_LOG_DIR = log/
ROSBRIDGE_ADDRESS ?= 127.0.0.1
ROSBRIDGE_PORT ?= 9090
ROSBRIDGE_SERVICE ?= openmower-rosbridge.service
SYSTEMD_USER_DIR ?= $(HOME)/.config/systemd/user
SHELL := /bin/bash

all: custom-deps deps build

.PHONY: deps custom-deps build-libs build build-release sim run dev run-foxglove rsp remote-devices rosbridge rosbridge-deps rosbridge-service-install rosbridge-service-enable rosbridge-service-disable rosbridge-service-restart rosbridge-service-status rosbridge-service-logs

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

rosbridge:
	ROS_DISTRO="$(ROS_DISTRO)" ROSBRIDGE_ADDRESS="$(ROSBRIDGE_ADDRESS)" ROSBRIDGE_PORT="$(ROSBRIDGE_PORT)" bash utils/run-rosbridge.sh

rosbridge-deps:
	sudo apt update
	sudo apt install -y "ros-$(ROS_DISTRO)-rosbridge-server"

rosbridge-service-install:
	install -d "$(SYSTEMD_USER_DIR)"
	sed -e 's|@WORKSPACE@|$(CURDIR)|g' -e 's|@ROS_DISTRO@|$(ROS_DISTRO)|g' -e 's|@ROSBRIDGE_ADDRESS@|$(ROSBRIDGE_ADDRESS)|g' -e 's|@ROSBRIDGE_PORT@|$(ROSBRIDGE_PORT)|g' systemd/openmower-rosbridge.service.in > "$(SYSTEMD_USER_DIR)/$(ROSBRIDGE_SERVICE)"
	systemctl --user daemon-reload
	@printf 'Installed %s in %s\n' "$(ROSBRIDGE_SERVICE)" "$(SYSTEMD_USER_DIR)"

rosbridge-service-enable: rosbridge-service-install
	systemctl --user enable --now "$(ROSBRIDGE_SERVICE)"

rosbridge-service-disable:
	systemctl --user disable --now "$(ROSBRIDGE_SERVICE)"

rosbridge-service-restart:
	systemctl --user restart "$(ROSBRIDGE_SERVICE)"

rosbridge-service-status:
	systemctl --user status "$(ROSBRIDGE_SERVICE)"

rosbridge-service-logs:
	journalctl --user -u "$(ROSBRIDGE_SERVICE)" -f

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
