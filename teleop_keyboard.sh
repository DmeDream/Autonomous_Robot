#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/d-dream/robot_ws/install/setup.bash
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
