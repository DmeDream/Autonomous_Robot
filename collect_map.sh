#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
mapping_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$mapping_script_dir/collect_map.py" "$@"
