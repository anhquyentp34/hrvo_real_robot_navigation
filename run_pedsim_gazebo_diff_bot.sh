#!/usr/bin/env bash
# Pedsim + Gazebo + diff_bot (gói diff_simulation; URDF: diff_description).
set -euo pipefail
WS="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
exec roslaunch diff_simulation pedsim_diff_bot.launch "$@"
