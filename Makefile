REMOTE_HOST ?= omdev.local
REMOTE_USER ?= openmower
ROS_DISTRO ?= jazzy
ROS_LOG_DIR = log/
OMDEV_HOST ?= $(REMOTE_HOST)
OMDEV_USER ?= $(USER)
OMDEV_SSH ?= $(OMDEV_USER)@$(OMDEV_HOST)
OMDEV_SSH_OPTS ?=
OMDEV_RSYNC_SSH ?= ssh $(OMDEV_SSH_OPTS)
OMDEV_WORKSPACE ?= /home/$(OMDEV_USER)/OpenMowerNext
OMDEV_MAP_DIR ?= /home/$(OMDEV_USER)/next
OMDEV_ENV_FILE ?= /home/$(OMDEV_USER)/omnext/openmower.env
OMDEV_IMAGE ?= docker.io/library/ros:jazzy
OMDEV_BUILD_IMAGE ?= $(OMDEV_IMAGE)
OMDEV_CONTAINER_WORKSPACE ?= /target_ws
OMDEV_FIELDS2COVER_BASE_PATHS ?= src/lib/fields2cover
OMDEV_FIELDS2COVER_PACKAGES ?= --packages-select fields2cover
OMDEV_LIB_BASE_PATHS ?= src/lib/vesc src/lib/micro_ros_agent src/lib/ntrip_client src/lib/fusioncore/compass_msgs src/lib/fusioncore/fusioncore_core src/lib/fusioncore/fusioncore_ros src/lib/ublox_f9p
OMDEV_LIB_PACKAGES ?= --packages-select vesc_msgs vesc_driver vesc_hw_interface vesc micro_ros_agent ntrip_client compass_msgs fusioncore_core fusioncore_ros ublox_f9p
OMDEV_APP_PACKAGES ?= --packages-select open_mower_next
OMDEV_CMAKE_ARGS ?= --cmake-args -DBUILD_TESTING=OFF
OMDEV_FIELDS2COVER_CMAKE_ARGS ?= --cmake-args -DBUILD_TESTING=OFF -DBUILD_TUTORIALS=OFF -DBUILD_PYTHON=OFF -DBUILD_DOC=OFF
OMDEV_LIB_CMAKE_ARGS ?= $(OMDEV_CMAKE_ARGS)
OMDEV_APP_CMAKE_ARGS ?= $(OMDEV_CMAKE_ARGS)
OMDEV_LAUNCH_ARGS ?=
OMDEV_CONTAINER ?= next-dev
OMDEV_PODMAN ?= sudo podman
OMDEV_TOOL_DEPS ?= python3-colcon-common-extensions python3-vcstool python3-rosdep
OMDEV_ROSDEP_PATHS ?= . $(OMDEV_FIELDS2COVER_BASE_PATHS) $(OMDEV_LIB_BASE_PATHS)
OMDEV_ROSDEP_SKIP_KEYS ?= webots_ros2_control webots_ros2_driver
OMDEV_LAUNCH_LOG ?= log/omdev-launch.log
OMDEV_LAUNCH_PID ?= /tmp/openmowernext-launch.pid
OMDEV_CLEAN_PATHS ?= build install log
OMDEV_RSYNC_DELETE ?=
OMDEV_RSYNC_EXCLUDES = \
	--exclude .git/ \
	--exclude '**/.git/' \
	--exclude build/ \
	--exclude install/ \
	--exclude log/ \
	--exclude .cache/ \
	--exclude .pytest_cache/ \
	--exclude docs/node_modules/ \
	--exclude docs/.vitepress/dist/
ROSBRIDGE_ADDRESS ?= 127.0.0.1
ROSBRIDGE_PORT ?= 9090
ROSBRIDGE_SERVICE ?= openmower-rosbridge.service
FOXGLOVE_ADDRESS ?= 0.0.0.0
FOXGLOVE_PORT ?= 8765
FOXGLOVE_SERVICE ?= openmower-foxglove.service
FOXGLOVE_USE_SIM_TIME ?= false
WEBOTS_STREAM ?= true
WEBOTS_PORT ?= 1234
SYSTEMD_USER_DIR ?= $(HOME)/.config/systemd/user
SHELL := /bin/bash

all: custom-deps deps build

.PHONY: deps custom-deps build-libs build build-release sim run calibrate dev run-foxglove foxglove foxglove-deps foxglove-service-install foxglove-service-enable foxglove-service-disable foxglove-service-restart foxglove-service-status foxglove-service-logs rsp remote-devices omdev omdev-sync omdev-pull omdev-dev-create omdev-dev-recreate omdev-dev-start omdev-dev-stop omdev-deps omdev-clean omdev-run omdev-run-workspace omdev-restart omdev-stop omdev-logs omdev-status omdev-shell omdev-build rosbridge rosbridge-deps rosbridge-service-install rosbridge-service-enable rosbridge-service-disable rosbridge-service-restart rosbridge-service-status rosbridge-service-logs

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
	bash -lc 'if [ -z "$${WEBOTS_HOME}" ] && [ -d "$${HOME}/.ros/webotsR2025a/webots" ]; then export WEBOTS_HOME="$${HOME}/.ros/webotsR2025a/webots"; fi && if [ -z "$${DISPLAY}" ] && [ -z "$${WEBOTS_OFFSCREEN}" ]; then export WEBOTS_OFFSCREEN=1; fi && source /opt/ros/$${ROS_DISTRO:-jazzy}/setup.bash && source install/setup.bash && set -a && source .devcontainer/default.env && set +a && ros2 launch open_mower_next sim.launch.py webots_stream:="$(WEBOTS_STREAM)" webots_port:="$(WEBOTS_PORT)"'

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

foxglove:
	ROS_DISTRO="$(ROS_DISTRO)" FOXGLOVE_ADDRESS="$(FOXGLOVE_ADDRESS)" FOXGLOVE_PORT="$(FOXGLOVE_PORT)" FOXGLOVE_USE_SIM_TIME="$(FOXGLOVE_USE_SIM_TIME)" bash utils/run-foxglove.sh

foxglove-deps:
	sudo apt update
	sudo apt install -y "ros-$(ROS_DISTRO)-foxglove-bridge"

foxglove-service-install:
	install -d "$(SYSTEMD_USER_DIR)"
	sed -e 's|@WORKSPACE@|$(CURDIR)|g' -e 's|@ROS_DISTRO@|$(ROS_DISTRO)|g' -e 's|@FOXGLOVE_ADDRESS@|$(FOXGLOVE_ADDRESS)|g' -e 's|@FOXGLOVE_PORT@|$(FOXGLOVE_PORT)|g' -e 's|@FOXGLOVE_USE_SIM_TIME@|$(FOXGLOVE_USE_SIM_TIME)|g' systemd/openmower-foxglove.service.in > "$(SYSTEMD_USER_DIR)/$(FOXGLOVE_SERVICE)"
	systemctl --user daemon-reload
	@printf 'Installed %s in %s\n' "$(FOXGLOVE_SERVICE)" "$(SYSTEMD_USER_DIR)"

foxglove-service-enable: foxglove-service-install
	systemctl --user enable --now "$(FOXGLOVE_SERVICE)"

foxglove-service-disable:
	systemctl --user disable --now "$(FOXGLOVE_SERVICE)"

foxglove-service-restart:
	systemctl --user restart "$(FOXGLOVE_SERVICE)"

foxglove-service-status:
	systemctl --user status "$(FOXGLOVE_SERVICE)"

foxglove-service-logs:
	journalctl --user -u "$(FOXGLOVE_SERVICE)" -f

run:
	ros2 launch launch/openmower.launch.py

calibrate:
	ros2 run open_mower_next calibrate_robot

dev:
	cd .devcontainer && docker-compose up -d

run-foxglove:
	$(MAKE) foxglove

rsp:
	ros2 launch launch/rsp.launch.py

remote-devices:
	bash .devcontainer/scripts/remote_devices.sh $(REMOTE_HOST) $(REMOTE_USER)

omdev: omdev-run-workspace

omdev-sync:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'mkdir -p "$(OMDEV_WORKSPACE)" "$(OMDEV_MAP_DIR)" "$$(dirname "$(OMDEV_ENV_FILE)")"'
	rsync -az --human-readable --info=progress2 $(OMDEV_RSYNC_DELETE) $(OMDEV_RSYNC_EXCLUDES) -e '$(OMDEV_RSYNC_SSH)' ./ "$(OMDEV_SSH):$(OMDEV_WORKSPACE)/"

omdev-pull:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) pull "$(OMDEV_IMAGE)"'

omdev-dev-create:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'set -e; mkdir -p "$(OMDEV_WORKSPACE)" "$(OMDEV_MAP_DIR)" "$$(dirname "$(OMDEV_ENV_FILE)")"; if ! $(OMDEV_PODMAN) container exists "$(OMDEV_CONTAINER)"; then env_args=""; if [ -f "$(OMDEV_ENV_FILE)" ]; then env_args="--env-file $(OMDEV_ENV_FILE)"; else echo "warning: $(OMDEV_ENV_FILE) not found; using image defaults for datum"; fi; $(OMDEV_PODMAN) pull "$(OMDEV_IMAGE)"; $(OMDEV_PODMAN) create --name "$(OMDEV_CONTAINER)" --privileged --network host --entrypoint /bin/bash -v /dev:/dev -v "$(OMDEV_WORKSPACE):$(OMDEV_CONTAINER_WORKSPACE)" -v "$(OMDEV_MAP_DIR):/next" -w "$(OMDEV_CONTAINER_WORKSPACE)" $$env_args -e OM_MAP_PATH=/next/map.json "$(OMDEV_IMAGE)" -lc "trap : TERM INT; sleep infinity & wait" >/dev/null; fi'

omdev-dev-recreate:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) rm -f -t 0 "$(OMDEV_CONTAINER)" >/dev/null 2>&1 || true'
	$(MAKE) omdev-dev-create

omdev-dev-start: omdev-dev-create
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'set -e; if [ "$$($(OMDEV_PODMAN) inspect -f "{{.State.Running}}" "$(OMDEV_CONTAINER)")" != "true" ]; then $(OMDEV_PODMAN) start "$(OMDEV_CONTAINER)" >/dev/null; fi'

omdev-dev-stop:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) stop "$(OMDEV_CONTAINER)" >/dev/null 2>&1 || true'

omdev-deps: omdev-sync omdev-dev-start
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'set -e; $(OMDEV_PODMAN) exec -u 0 "$(OMDEV_CONTAINER)" bash -lc "set -e; apt-get update; DEBIAN_FRONTEND=noninteractive apt-get install -y $(OMDEV_TOOL_DEPS); rosdep init >/dev/null 2>&1 || true; rosdep update; source /opt/ros/$(ROS_DISTRO)/setup.bash; cd $(OMDEV_CONTAINER_WORKSPACE); DEBIAN_FRONTEND=noninteractive rosdep install --from-paths $(OMDEV_ROSDEP_PATHS) --ignore-src -i -y -r --skip-keys=\"$(OMDEV_ROSDEP_SKIP_KEYS)\""'

omdev-clean: omdev-dev-start
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) exec -u 0 "$(OMDEV_CONTAINER)" bash -lc "cd $(OMDEV_CONTAINER_WORKSPACE) && rm -rf $(OMDEV_CLEAN_PATHS)"'

omdev-run: omdev-run-workspace

omdev-run-workspace: omdev-sync omdev-dev-start
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'env_args=""; if [ -f "$(OMDEV_ENV_FILE)" ]; then env_args="--env-file $(OMDEV_ENV_FILE)"; fi; $(OMDEV_PODMAN) exec $$env_args "$(OMDEV_CONTAINER)" bash -lc "cd $(OMDEV_CONTAINER_WORKSPACE) && bash utils/omdev-run-launch.sh $(ROS_DISTRO) $(OMDEV_LAUNCH_LOG) $(OMDEV_LAUNCH_PID) $(OMDEV_LAUNCH_ARGS)"'

omdev-restart: omdev-run-workspace

omdev-stop:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) exec "$(OMDEV_CONTAINER)" bash -lc "cd $(OMDEV_CONTAINER_WORKSPACE) && bash utils/omdev-stop-launch.sh $(OMDEV_LAUNCH_PID)"'

omdev-logs: omdev-dev-start
	ssh -t $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) exec -it "$(OMDEV_CONTAINER)" bash -lc "cd $(OMDEV_CONTAINER_WORKSPACE) && touch $(OMDEV_LAUNCH_LOG) && tail -n 200 -f $(OMDEV_LAUNCH_LOG)"'

omdev-status:
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'hostname; id; test -f "$(OMDEV_ENV_FILE)" && echo "env: $(OMDEV_ENV_FILE)" || echo "env missing: $(OMDEV_ENV_FILE)"; test -f "$(OMDEV_MAP_DIR)/map.json" && echo "map: $(OMDEV_MAP_DIR)/map.json" || echo "map missing: $(OMDEV_MAP_DIR)/map.json"; $(OMDEV_PODMAN) ps -a --filter name="^$(OMDEV_CONTAINER)$$"; $(OMDEV_PODMAN) images "$(OMDEV_IMAGE)"'

omdev-shell:
	ssh -t $(OMDEV_SSH_OPTS) $(OMDEV_SSH) '$(OMDEV_PODMAN) exec -it "$(OMDEV_CONTAINER)" bash'

omdev-build: omdev-sync omdev-dev-start
	ssh $(OMDEV_SSH_OPTS) $(OMDEV_SSH) 'set -e; uid=$$(id -u); gid=$$(id -g); $(OMDEV_PODMAN) exec --user "$$uid:$$gid" -e HOME=/tmp "$(OMDEV_CONTAINER)" bash -lc "source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(OMDEV_CONTAINER_WORKSPACE) && colcon build --symlink-install --base-paths $(OMDEV_FIELDS2COVER_BASE_PATHS) $(OMDEV_FIELDS2COVER_PACKAGES) $(OMDEV_FIELDS2COVER_CMAKE_ARGS) && source install/setup.bash && colcon build --symlink-install --base-paths $(OMDEV_LIB_BASE_PATHS) $(OMDEV_LIB_PACKAGES) $(OMDEV_LIB_CMAKE_ARGS) && source install/setup.bash && colcon build --symlink-install $(OMDEV_APP_PACKAGES) $(OMDEV_APP_CMAKE_ARGS)"'
