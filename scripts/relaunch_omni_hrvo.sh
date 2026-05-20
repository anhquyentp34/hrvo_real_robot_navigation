#!/usr/bin/env bash
# Dọn phiên ROS/Gazebo cũ rồi chạy lại (tránh node trùng tên / gzserver segfault).
set -e
source /opt/ros/noetic/setup.bash
source "$(dirname "$0")/../devel/setup.bash"

echo "Dừng roslaunch / Gazebo cũ..."
pkill -9 -f "roslaunch x_omni4wd_simulation x_omni4wd_hrvo" 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f rosmaster 2>/dev/null || true
sleep 2

echo "Khởi động x_omni4wd_hrvo_simulation (staged_startup mặc định)..."
exec roslaunch x_omni4wd_simulation x_omni4wd_hrvo_simulation.launch "$@"
