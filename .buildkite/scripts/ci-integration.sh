#!/usr/bin/env bash

set -euo pipefail

docker run --rm \
  --volume "$PWD:/workspace" \
  --workdir /workspace \
  --env ROS_DISTRO=jazzy \
  --env DEBIAN_FRONTEND=noninteractive \
  --env WEBOTS_OFFSCREEN=1 \
  ros:jazzy \
  bash -lc '
    set -euo pipefail

    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      bzip2 \
      ca-certificates \
      cmake \
      curl \
      git \
      libxcb-cursor0 \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      xvfb

    rosdep init >/dev/null 2>&1 || true
    rosdep update

    webots_root="${HOME}/.ros/webotsR2025a"
    webots_home="${webots_root}/webots"
    if [ ! -x "${webots_home}/webots" ]; then
      rm -rf "${webots_root}"
      mkdir -p "${webots_root}"
      curl -L -o /tmp/webots-R2025a-x86-64.tar.bz2 \
        https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a-x86-64.tar.bz2
      tar -xjf /tmp/webots-R2025a-x86-64.tar.bz2 -C "${webots_root}"
    fi
    test -x "${webots_home}/webots"

    export WEBOTS_HOME="${webots_home}"
    export WEBOTS_OFFSCREEN=1

    source /opt/ros/${ROS_DISTRO}/setup.bash
    make custom-deps deps

    make build-libs
    source install/setup.bash
    colcon build \
      --symlink-install \
      --packages-select open_mower_next \
      --cmake-args -DOPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS=ON

    source install/setup.bash
    colcon test \
      --packages-select open_mower_next \
      --event-handlers console_direct+ \
      --ctest-args -L integration --output-on-failure
    colcon test-result \
      --test-result-base build/open_mower_next/test_results \
      --verbose
  '
