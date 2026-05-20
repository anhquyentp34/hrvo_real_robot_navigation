#!/usr/bin/env bash
# Chạy mô phỏng Pedsim + Gazebo từ workspace này (thay cho pedsim_ws).
# Mặc định: không spawn robot Kobuki; bỏ with_robot:=false nếu truyền tham số khác qua "$@".
set -euo pipefail
WS="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
exec roslaunch pedsim_simulator robot.launch with_robot:=false "$@"
