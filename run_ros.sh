#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
room_run_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '[g]z sim -s|[r]os2 launch .*simulation.launch.py' >/dev/null; then
    echo 'A simulation is already running. Stop its bringup before running run_ros.sh.'
    exit 1
fi
exec ros2 launch "$room_run_dir/room_simulation.launch.py" "$@"
