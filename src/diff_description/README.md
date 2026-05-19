# diff_description

Goi mo ta robot `diff_bot` (URDF/xacro) va launch hien thi nhanh trong Gazebo + RViz.

## Tep chinh

- `urdf/diff_bot.urdf.xacro`: model robot (diff drive + Velodyne VLP-16 + view camera).
- `launch/diff_bot_gazebo_rviz.launch`: launch duy nhat de spawn robot trong Gazebo va mo RViz.
- `rviz/gazebo_spawn_model.rviz`: cau hinh RViz mac dinh.

## Chay nhanh

```bash
source ~/hrvo_real_robot_navigation/devel/setup.bash
roslaunch diff_description diff_bot_gazebo_rviz.launch
```

## Tham so hay dung

- `gpu:=false|true`: chon topic cloud Velodyne (`_cpu` hoac `_gpu`).
- `velodyne_visualize:=true|false`: bat/tat tia quet Velodyne trong Gazebo.
- `use_rviz:=true|false`: bat/tat RViz.
- `x`, `y`, `z`, `yaw`: vi tri spawn.

## Ghi chu

- Plugin dieu khien la `libgazebo_ros_diff_drive` trong URDF, nhan lenh tu `/cmd_vel`.
- Odom mac dinh publish ra `/odom`, frame `base_footprint`.
