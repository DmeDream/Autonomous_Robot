#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
auto_mapping_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$auto_mapping_dir/auto_map.py" "$@"
