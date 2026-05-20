# HƯỚNG DẪN SỬ DỤNG GAZEBO SLAM

## Tổng quan

Package `x_omni4wd_gazebo` cung cấp launch file tích hợp hoàn chỉnh cho SLAM simulation trong Gazebo.

## Launch File: `gazebo_slam.launch`

Launch file này tích hợp:
1. **Gazebo simulation** với robot và controllers
2. **RF2O Laser Odometry** (publish `odom -> base_footprint`)
3. **Hector SLAM** (publish `map -> odom`)
4. **RViz visualization**

### TF Tree

```
map -> odom -> base_footprint -> base_link -> lidar_link
```

### Topics

- `/scan`: Laser scan data (từ Gazebo)
- `/odom`: Odometry từ RF2O (`nav_msgs/Odometry`)
- `/map`: Occupancy grid map từ Hector (`nav_msgs/OccupancyGrid`)
- `/cmd_vel`: Velocity commands (`geometry_msgs/Twist`)

## Cách sử dụng

### 1. Basic Usage

```bash
source /opt/ros/noetic/setup.bash
source ~/costmapex_unified_ws/devel/setup.bash

roslaunch x_omni4wd_gazebo gazebo_slam.launch
```

### 2. Với các tùy chọn

```bash
# Không mở RViz
roslaunch x_omni4wd_gazebo gazebo_slam.launch use_rviz:=false

# Không mở rqt_robot_steering
roslaunch x_omni4wd_gazebo gazebo_slam.launch use_rqt_robot_steering:=false

# Thay đổi vị trí spawn robot
roslaunch x_omni4wd_gazebo gazebo_slam.launch x:=1.0 y:=2.0 z:=0.1
```

### 3. Arguments

| Argument | Default | Mô tả |
|----------|---------|-------|
| `world` | `empty` | World name |
| `world_file` | `$(find x_omni4wd_gazebo)/worlds/empty.world` | Path to world file |
| `paused` | `false` | Start Gazebo paused |
| `use_sim_time` | `true` | Use simulation time |
| `gui` | `true` | Show Gazebo GUI |
| `headless` | `false` | Run Gazebo headless |
| `x`, `y`, `z` | `0.0`, `0.0`, `0.1` | Robot spawn position |
| `use_rviz` | `true` | Launch RViz |
| `use_rqt_robot_steering` | `true` | Launch rqt_robot_steering |
| `scan_topic` | `/scan` | Topic laser cho Hector |
| `odom_frame` | `odom` | Frame odometry |
| `base_frame` | `base_footprint` | Frame base robot |
| `rviz_config` | `$(find x_omni4wd_gazebo)/rviz/slam.rviz` | File cấu hình RViz |

## Điều khiển robot

### Sử dụng rqt_robot_steering

Sau khi launch, mở `rqt_robot_steering` để điều khiển robot:
- **Linear X**: Tiến/lùi
- **Angular Z**: Quay trái/phải

### Sử dụng cmd_vel topic

```bash
# Di chuyển tiến
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# Quay trái
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10

# Dừng
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
```

## Kiểm tra hệ thống

### 1. Kiểm tra nodes

```bash
rosnode list | grep -E "hector|rf2o|gazebo"
```

### 2. Kiểm tra topics

```bash
rostopic list | grep -E "scan|odom|map|cmd_vel"
```

### 3. Kiểm tra TF tree

```bash
# View TF tree
rosrun tf view_frames

# Check specific transform
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
```

### 4. Kiểm tra map

```bash
# Xem thông tin map
rostopic echo /map -n 1

# Kiểm tra map đang được publish
rostopic hz /map
```

## Lưu map

Sau khi mapping xong, lưu map dạng PGM/YAML (map_server):

```bash
rosrun map_server map_saver -f /path/to/map_name
```

## Troubleshooting

### 1. Hector không publish map

- Kiểm tra RF2O có publish `/odom` không: `rostopic hz /odom`
- Kiểm tra laser scan: `rostopic hz /scan`
- Kiểm tra TF tree: `rosrun tf view_frames`

### 2. Robot không di chuyển

- Kiểm tra controllers: `rosservice list | grep controller`
- Kiểm tra joint states: `rostopic echo /x_omni4wd/joint_states`
- Kiểm tra cmd_vel: `rostopic echo /cmd_vel`

### 3. TF errors

- Đảm bảo `base_footprint -> base_link` transform được publish
- Kiểm tra RF2O có publish `odom -> base_footprint` không
- Tham số Hector: `config/hector_x_omni4wd.yaml` trong `x_omni4wd_slam`

## So sánh với các launch files khác

| Launch File | Mô tả |
|-------------|-------|
| `gazebo_slam.launch` | **Tích hợp đầy đủ**: Gazebo + RF2O + Hector + RViz |
| `x_omni4wd_control/gazebo_velocity_control.launch` | Chỉ Gazebo + ROS Control |
| `x_omni4wd_slam/rf2o_laser_odometry.launch` | Gazebo + RF2O (không có SLAM map) |
| `x_omni4wd_slam/x_omni4wd_hector_mapping.launch` | Robot thật + Hector (bringup + LiDAR) |

## Tích hợp với gazebo_simulation (ROS 2)

Launch file này được tạo dựa trên cấu trúc của `gazebo_simulation` package trong `omni_navigation` (ROS 2), nhưng được port sang ROS 1:

- **Tương đồng**: Cấu trúc tích hợp, TF tree, topics
- **Khác biệt**: ROS 1 XML launch thay vì ROS 2 Python launch, không có Nav2 stack

## Tài liệu tham khảo

- [Hector SLAM](http://wiki.ros.org/hector_mapping)
- [RF2O Laser Odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [Gazebo ROS](http://gazebosim.org/tutorials?cat=connect_ros)
