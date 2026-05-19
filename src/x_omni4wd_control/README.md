# x_omni4wd_control

ROS Control package for x_omni4wd X-drive omniwheel robot with wheel-level kinematics.

## 📋 Tổng quan

Package này cung cấp bộ điều khiển wheel-level cho robot 4 bánh omniwheel với cấu hình X-drive, sử dụng ROS Control và kinematics chính xác cho omniwheel (không phải mecanum).

## 🏗️ Kiến trúc

```
/cmd_vel (geometry_msgs/Twist)
    ↓
xdrive_kinematics_node
    ↓ Inverse Kinematics (X-drive omniwheel)
    ↓
ROS Control Topics:
  - /x_omni4wd/front_left_wheel_velocity_controller/command
  - /x_omni4wd/rear_left_wheel_velocity_controller/command
  - /x_omni4wd/rear_right_wheel_velocity_controller/command
  - /x_omni4wd/front_right_wheel_velocity_controller/command
    ↓
ROS Control (gazebo_ros_control)
    ↓ JointVelocityController
    ↓
Gazebo Joints
    ↓
/joint_states (feedback)
    ↓
xdrive_kinematics_node
    ↓ Forward Kinematics
    ↓
/odom (nav_msgs/Odometry)
TF: odom → base_link
```

## 🔧 Cài đặt và Build

### 1. Build package

```bash
cd ~/x_omni4wd_ws
catkin_make
source devel/setup.bash
```

### 2. Kiểm tra dependencies

Package yêu cầu:
- ROS Noetic
- `eigen3` (sudo apt-get install libeigen3-dev)
- `velocity_controllers` (sudo apt-get install ros-noetic-velocity-controllers)

## 🚀 Sử dụng

### Chạy simulation với ROS Control

```bash
roslaunch x_omni4wd_control x_omni4wd_ros_control.launch
```

### Điều khiển robot

#### Cách 1: Keyboard teleop

```bash
# Terminal 2
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### Cách 2: Command line

```bash
# Di chuyển về phía trước
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Di chuyển sang trái
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.5, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Quay tại chỗ (CCW)
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 1.0}"

# Dừng
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## 🔍 Kiểm tra

### Kiểm tra odometry

```bash
rostopic echo /odom
```

### Kiểm tra TF tree

```bash
rosrun tf view_frames
evince frames.pdf
```

Hoặc trong RViz:
- Add TF display
- Kiểm tra transform từ `odom` đến `base_link`

### Kiểm tra wheel commands

```bash
rostopic echo /x_omni4wd/front_left_wheel_velocity_controller/command
rostopic echo /x_omni4wd/rear_left_wheel_velocity_controller/command
rostopic echo /x_omni4wd/rear_right_wheel_velocity_controller/command
rostopic echo /x_omni4wd/front_right_wheel_velocity_controller/command
```

### Kiểm tra joint states

```bash
rostopic echo /x_omni4wd/joint_states
```

### Kiểm tra controllers

```bash
rosservice call /x_omni4wd/controller_manager/list_controllers
```

## 🔗 Tích hợp với costmapex_unified_ws (diffbot_navigation)

Trong workspace **costmapex_unified_ws**, `diffbot_navigation` có thể chạy robot **x_omni4wd** thay Diffbot (arg `robot_type:=x_omni4wd`). Khi đó:

- **Topic:** move_base gửi `/cmd_vel` → node `cmd_vel_to_omni_wheels` (x_omni4wd_control) chuyển thành lệnh bánh; odometry dùng `/odom` (từ RF2O trong launch, không dùng kinematics node khi có RF2O).
- **Goal_sender:** Tương thích cả diffbot và x_omni4wd (cùng move_base, frame map). Cấu hình move_base/TEB cho x_omni4wd đã được chỉnh để move_base báo SUCCEEDED ổn định (xem `diffbot_navigation/docs/TOM_TAT_BAI_HOC_X_OMNI4WD_GOAL.md`).

Cần source overlay x_omni4wd_ws (hoặc build costmapex_unified_ws có dependency x_omni4wd_*) trước khi chạy launch so sánh.

## ⚙️ Cấu hình

### Tham số node

Các tham số có thể điều chỉnh trong launch file:

- `wheel_radius`: Bán kính bánh (mặc định: 0.05 m)
- `odom_frame`: Frame cho odometry (mặc định: "odom")
- `base_frame`: Base frame (mặc định: "base_link")
- `publish_tf`: Có publish TF không (mặc định: true)
- `cmd_timeout`: Timeout cho cmd_vel (s) (mặc định: 0.2)
- `sign_fl`, `sign_rl`, `sign_rr`, `sign_fr`: Sign calibration per wheel (mặc định: 1.0)

### Điều chỉnh sign nếu bị ngược

Nếu robot di chuyển ngược hướng mong muốn, sửa trong launch file:

```xml
<param name="sign_fl" value="-1.0" />  <!-- Đảo dấu front left -->
```

Hoặc tạo file config riêng và load:

```yaml
# config/sign_calibration.yaml
xdrive_kinematics_node:
  sign_fl: -1.0
  sign_rl: 1.0
  sign_rr: 1.0
  sign_fr: -1.0
```

Load trong launch:
```xml
<rosparam file="$(find x_omni4wd_control)/config/sign_calibration.yaml" command="load" />
```

## 🧪 Test 3 chuyển động cơ bản

### 1. Test tiến (linear.x > 0)

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.3, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Kỳ vọng:**
- Robot di chuyển về phía trước (theo hướng +x của base_link)
- Tất cả 4 bánh quay cùng chiều (có thể khác tốc độ do X-configuration)

**Nếu ngược:** Đảo dấu tất cả `sign_*` trong launch file.

### 2. Test dịch trái (linear.y > 0)

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.3, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Kỳ vọng:**
- Robot dịch sang trái (theo hướng +y của base_link)
- Bánh trái và phải quay ngược chiều nhau

**Nếu ngược:** Đảo dấu `sign_rl` và `sign_rr`, hoặc `sign_fl` và `sign_fr`.

### 3. Test quay CCW (angular.z > 0)

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 1.0}"
```

**Kỳ vọng:**
- Robot quay ngược chiều kim đồng hồ (CCW)
- Bánh trái và phải quay ngược chiều nhau

**Nếu ngược:** Đảo dấu `angular.z` trong cmd_vel hoặc điều chỉnh sign cho các bánh.

## 📐 Kinematics

### Inverse Kinematics

Từ vận tốc robot `[vx, vy, ω]^T` tính vận tốc từng bánh:

```
v_i = [vx, vy]^T + ω * [-y_i, x_i]^T
wheel_speed_i = (1/r) * d_i^T * v_i
```

Trong đó:
- `d_i = [cos(alpha_i), sin(alpha_i)]` là hướng lăn hiệu dụng
- `alpha_i` là góc từ joint rpy yaw
- `(x_i, y_i)` là vị trí bánh trong base_link

### Forward Kinematics

Từ vận tốc bánh tính vận tốc robot bằng least squares:

```
A * [vx, vy, ω]^T = b
```

Giải bằng pseudo-inverse: `x = (A^T * A)^(-1) * A^T * b`

## 📁 Cấu trúc Package

```
x_omni4wd_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── x_omni4wd_control.yaml
├── launch/
│   └── x_omni4wd_ros_control.launch
└── src/
    └── xdrive_kinematics_node.cpp
```

## 🔗 Dependencies

- `x_omni4wd_description`: Robot URDF/Xacro
- `gazebo_ros`: Gazebo integration
- `gazebo_ros_control`: ROS Control plugin
- `controller_manager`: Controller management
- `velocity_controllers`: JointVelocityController
- `eigen3`: Matrix operations

## ⚠️ Lưu ý

1. **Sign calibration:** Luôn test 3 chuyển động cơ bản và điều chỉnh sign nếu cần
2. **Wheel radius:** Đảm bảo khớp với URDF (0.05 m)
3. **Joint names:** Phải khớp với xacro (`base_link_*_wheel_joint`)
4. **Namespace:** Mặc định `/x_omni4wd`, có thể override bằng arg `robot_ns`

## 🐛 Troubleshooting

### Controllers không spawn

```bash
# Kiểm tra config
rosparam list | grep x_omni4wd

# Kiểm tra controllers
rosservice call /x_omni4wd/controller_manager/list_controllers
```

### Robot không di chuyển

1. Kiểm tra cmd_vel: `rostopic echo /cmd_vel`
2. Kiểm tra wheel commands: `rostopic echo /x_omni4wd/*/command`
3. Kiểm tra joint states: `rostopic echo /x_omni4wd/joint_states`
4. Kiểm tra sign calibration

### Odometry không đúng

1. Kiểm tra forward kinematics trong code
2. Kiểm tra wheel feedback từ joint_states
3. Kiểm tra TF: `rosrun tf view_frames`

---

**Tác giả:** Senior Robotics Engineer  
**ROS Version:** Noetic  
**Gazebo:** Classic

