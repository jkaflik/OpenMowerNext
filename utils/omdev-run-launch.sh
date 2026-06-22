#!/usr/bin/env bash
set -euo pipefail

ros_distro="$1"
launch_log="$2"
pid_file="$3"
shift 3

export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

bash utils/omdev-stop-launch.sh "$pid_file" >/dev/null 2>&1 || true

mkdir -p "$(dirname "$launch_log")"
: > "$launch_log"

setsid bash -c '
  set -euo pipefail
  ros_distro="$1"
  shift
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
  set +u
  source "/opt/ros/${ros_distro}/setup.bash"
  source install/setup.bash
  set -u
  exec ros2 launch open_mower_next openmower.launch.py "$@"
' omdev-launch "$ros_distro" "$@" >> "$launch_log" 2>&1 &

pid="$!"
printf '%s\n' "$pid" > "$pid_file"
printf 'Started OpenMowerNext launch pid %s; log: %s\n' "$pid" "$launch_log"
