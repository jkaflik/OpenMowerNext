#!/usr/bin/env bash
set -euo pipefail

pid_file="${1:-/tmp/openmowernext-launch.pid}"

if [ ! -f "$pid_file" ]; then
  printf 'No OpenMowerNext launch pid file at %s\n' "$pid_file"
  exit 0
fi

pid=""
read -r pid < "$pid_file" || true
rm -f "$pid_file"

if [ -z "$pid" ]; then
  printf 'OpenMowerNext launch pid file was empty\n'
  exit 0
fi

if ! kill -0 "$pid" 2>/dev/null; then
  printf 'OpenMowerNext launch pid %s is not running\n' "$pid"
  exit 0
fi

kill_tree() {
  local parent="$1"
  local child
  for child in $(pgrep -P "$parent" 2>/dev/null || true); do
    kill_tree "$child"
    kill -TERM "$child" 2>/dev/null || true
  done
}

kill -TERM -- "-$pid" 2>/dev/null || true
kill_tree "$pid"
kill -TERM "$pid" 2>/dev/null || true
sleep 2
if kill -0 "$pid" 2>/dev/null; then
  kill -KILL -- "-$pid" 2>/dev/null || true
  kill_tree "$pid"
  kill -KILL "$pid" 2>/dev/null || true
fi

printf 'Stopped OpenMowerNext launch pid %s\n' "$pid"
