# diff_simulation

Goi mo phong tong hop cho `diff_bot`, hoc theo bo cuc `x_omni4wd_simulation`.

## Launch chinh

- `launch/diff_bot_simulation.launch`: world + spawn robot + velodyne->scan + AMCL + move_base + RViz.
  Luong AMCL/DWA duoc noi bo trong `diff_simulation` (khong can include `robot_gazebo`).
- `launch/pedsim_diff_bot.launch`: Pedsim + Gazebo + spawn `diff_bot`.
- `scenarios/f21_wall_full_pedsim.xml`: kich ban actor da canh chinh de di trong vung wall cua `f21_wall_full`.

## Tai nguyen world/map

Da bo sung world/map theo VCCA2026:

- `diff_gazebo/worlds/f21_wall_full.world`
- `diff_slam/maps/f21_wall_full_sim.yaml`
- `diff_slam/maps/f21_wall_full_sim.pgm`

## Chay nhanh

```bash
source ~/hrvo_real_robot_navigation/devel/setup.bash
roslaunch diff_simulation diff_bot_simulation.launch
```

## Tham so quan trong

- `local_planner:=teb|dwa|hrvo`
- `enable_pedsim:=true|false`
- `pedsim_scene_file:=...`
- `gpu:=false|true`
- `velodyne_visualize:=true|false`
- `world_name`, `map_file`
- `x_pos`, `y_pos`, `yaw`

## Vi du

```bash
# TEB + Velodyne GPU
roslaunch diff_simulation diff_bot_simulation.launch local_planner:=teb gpu:=true

# DWA
roslaunch diff_simulation diff_bot_simulation.launch local_planner:=dwa
```
