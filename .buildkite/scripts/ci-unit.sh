#!/usr/bin/env bash

set -euo pipefail

docker run --rm \
  --volume "$PWD:/workspace" \
  --workdir /workspace \
  --env ROS_DISTRO=jazzy \
  --env DEBIAN_FRONTEND=noninteractive \
  ros:jazzy \
  bash -lc '
    set -euo pipefail

    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool

    rosdep init >/dev/null 2>&1 || true
    rosdep update

    source /opt/ros/${ROS_DISTRO}/setup.bash
    make custom-deps deps

    make build-libs

    source install/setup.bash
    colcon build --symlink-install --packages-select open_mower_next

    source install/setup.bash
    colcon test \
      --packages-select open_mower_next \
      --event-handlers console_direct+ \
      --ctest-args -LE integration --output-on-failure
    colcon test-result \
      --test-result-base build/open_mower_next/test_results \
      --verbose
  '
