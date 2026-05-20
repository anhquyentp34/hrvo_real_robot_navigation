# x_omni4wd_gazebo

Package Gazebo simulation cho robot 4 bánh omni-directional với bố trí chữ X.

## Sử dụng

### Chạy simulation trong Gazebo

```bash
roslaunch x_omni4wd_gazebo x_omni4wd_world.launch
```

### Tùy chọn launch

```bash
# Không có GUI
roslaunch x_omni4wd_gazebo x_omni4wd_world.launch gui:=false

# Headless mode
roslaunch x_omni4wd_gazebo x_omni4wd_world.launch headless:=true
```

## Điều khiển Robot

### Gửi lệnh vận tốc

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Kiểm tra odometry

```bash
rostopic echo /odom
```

## Gazebo Plugin

Plugin `x_omni_force_based_move` điều khiển robot bằng lực:
- Subscribe: `/cmd_vel` (geometry_msgs/Twist)
- Publish: `/odom` (nav_msgs/Odometry)
- Hỗ trợ di chuyển đa hướng: linear.x, linear.y, angular.z

## Dependencies

- gazebo_ros
- gazebo_plugins
- x_omni4wd_description
- roscpp
- geometry_msgs
- nav_msgs
- tf

