#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
bringup_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '[r]os2 launch robot_bringup simulation.launch.py' >/dev/null; then
    echo 'Gazebo bringup is already running. Use its teleop window and run collect_map.sh.'
    exit 1
fi
if ! command -v terminator >/dev/null; then
    echo 'Terminal app terminator is required to open the teleop window.'
    exit 1
fi
terminator --no-dbus --title='Robot Teleop - i:forward ,:reverse j/l:turn k:stop' \
    -x bash "$bringup_script_dir/teleop_keyboard.sh" &
bringup_terminal_pid=$!
cleanup_bringup_terminal() {
    kill -TERM "$bringup_terminal_pid" 2>/dev/null || true
}
trap cleanup_bringup_terminal EXIT
echo 'Gazebo + keyboard teleop started. In another terminal: bash collect_map.sh'
ros2 launch robot_bringup simulation.launch.py rviz:=false
