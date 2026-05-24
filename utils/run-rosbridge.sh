#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROSBRIDGE_ADDRESS="${ROSBRIDGE_ADDRESS:-127.0.0.1}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${repo_root}/install/setup.bash" ]]; then
  source "${repo_root}/install/setup.bash"
fi
set -u

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:="${ROSBRIDGE_ADDRESS}" \
  port:="${ROSBRIDGE_PORT}"
