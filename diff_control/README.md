# diff_control

Goi dieu khien toc do cho `diff_bot` trong Gazebo.

## Chuc nang

- Chay mo phong voi `diff_description`.
- Chon backend teleop:
  - `teleop_twist_keyboard`
  - `diff_keyboard_teleop` (pynput)
- Co `cmd_vel_gate` de gioi han toc do va timeout an toan:
  - `teleop -> /cmd_vel_raw -> cmd_vel_gate -> /cmd_vel`.

## Tep chinh

- `launch/gazebo_diff_velocity.launch`
- `scripts/cmd_vel_gate.py`
- `config/diff_drive_limits.yaml`

## Chay nhanh

```bash
source ~/hrvo_real_robot_navigation/devel/setup.bash
roslaunch diff_control gazebo_diff_velocity.launch
```

## Lua chon teleop

```bash
# Teleop terminal (mac dinh)
roslaunch diff_control gazebo_diff_velocity.launch teleop_backend:=twist

# Teleop ban phim pynput
roslaunch diff_control gazebo_diff_velocity.launch teleop_backend:=keyboard
```

## Tham so quan trong

- `use_cmd_vel_gate:=true|false`
- `teleop:=true|false`
- `teleop_max_vx`, `teleop_max_wz`
- `gpu`, `velodyne_visualize`
