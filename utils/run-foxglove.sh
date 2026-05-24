#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
FOXGLOVE_USE_SIM_TIME="${FOXGLOVE_USE_SIM_TIME:-false}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${repo_root}/install/setup.bash" ]]; then
  source "${repo_root}/install/setup.bash"
fi
set -u

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:="${FOXGLOVE_ADDRESS}" \
  port:="${FOXGLOVE_PORT}" \
  include_hidden:="true" \
  use_sim_time:="${FOXGLOVE_USE_SIM_TIME}"
