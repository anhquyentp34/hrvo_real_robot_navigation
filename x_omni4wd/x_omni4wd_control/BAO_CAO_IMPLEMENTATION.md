# BÁO CÁO IMPLEMENTATION: x_omni4wd_control

## 📋 (1) CÂY FILE ĐÃ TẠO

```
x_omni4wd_control/
├── CMakeLists.txt                    # Build config với Eigen3 support
├── package.xml                       # Dependencies: roscpp, geometry_msgs, sensor_msgs, nav_msgs, tf2_ros, controller_manager_msgs, velocity_controllers
├── README.md                         # Hướng dẫn đầy đủ
├── CAY_FILE.md                       # Cây file
│
├── config/
│   └── x_omni4wd_control.yaml       # ROS Control config:
│                                     #   - joint_state_controller
│                                     #   - 4 velocity_controllers/JointVelocityController
│                                     #   - PID gains (p=10.0, i=0.0, d=0.1)
│                                     #   - gazebo_ros_control PID gains
│
├── launch/
│   └── x_omni4wd_ros_control.launch # Launch file:
│                                     #   - Load xacro với use_ros_control:=true
│                                     #   - Spawn robot vào Gazebo
│                                     #   - Spawn controllers
│                                     #   - Run xdrive_kinematics_node
│
└── src/
    └── xdrive_kinematics_node.cpp    # C++ node (336 dòng):
                                       #   - Inverse kinematics (cmd_vel → wheel speeds)
                                       #   - Forward kinematics (wheel feedback → odom)
                                       #   - Odometry publisher
                                       #   - TF broadcaster (odom → base_link)
                                       #   - Safety timeout
```

**Tổng:** 6 files

---

## 📝 (2) NỘI DUNG CÁC FILE CHÍNH

### File 1: `package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>x_omni4wd_control</name>
  <version>1.0.0</version>
  <description>ROS Control package for x_omni4wd X-drive omniwheel robot</description>
  
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>controller_manager_msgs</depend>
  <depend>controller_manager</depend>
  <depend>joint_state_controller</depend>
  <depend>velocity_controllers</depend>
  <depend>x_omni4wd_description</depend>
  <depend>gazebo_ros</depend>
  <depend>gazebo_ros_control</depend>
</package>
```

### File 2: `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs sensor_msgs nav_msgs
  tf2_ros tf2_geometry_msgs controller_manager_msgs
  controller_manager joint_state_controller velocity_controllers
  x_omni4wd_description gazebo_ros gazebo_ros_control
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES x_omni4wd_control
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(xdrive_kinematics_node src/xdrive_kinematics_node.cpp)
target_link_libraries(xdrive_kinematics_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS xdrive_kinematics_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config/ launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### File 3: `config/x_omni4wd_control.yaml`

```yaml
x_omni4wd:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50

  front_left_wheel_velocity_controller:
    type: velocity_controllers/JointVelocityController
    joint: base_link_front_left_wheel_joint
    pid: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}

  rear_left_wheel_velocity_controller:
    type: velocity_controllers/JointVelocityController
    joint: base_link_rear_left_wheel_joint
    pid: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}

  rear_right_wheel_velocity_controller:
    type: velocity_controllers/JointVelocityController
    joint: base_link_rear_right_wheel_joint
    pid: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}

  front_right_wheel_velocity_controller:
    type: velocity_controllers/JointVelocityController
    joint: base_link_front_right_wheel_joint
    pid: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}

  gazebo_ros_control:
    pid_gains:
      base_link_front_left_wheel_joint: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}
      base_link_rear_left_wheel_joint: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}
      base_link_rear_right_wheel_joint: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}
      base_link_front_right_wheel_joint: {p: 10.0, i: 0.0, d: 0.1, i_clamp_min: -100, i_clamp_max: 100}
```

### File 4: `launch/x_omni4wd_ros_control.launch`

```xml
<?xml version="1.0"?>
<launch>
  <arg name="robot_ns" default="/x_omni4wd" />
  <arg name="use_sim_time" default="true" />
  <arg name="gui" default="true" />

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="gui" value="$(arg gui)" />
    <arg name="use_sim_time" value="$(arg use_sim_time)" />
  </include>

  <param name="robot_description" 
         command="$(find xacro)/xacro $(find x_omni4wd_description)/urdf/x_omni4wd.xacro use_ros_control:=true" />

  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -model x_omni4wd -param robot_description -x 0 -y 0 -z 0.5" />

  <rosparam file="$(find x_omni4wd_control)/config/x_omni4wd_control.yaml" command="load" ns="$(arg robot_ns)" />

  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="$(arg robot_ns)"
        args="joint_state_controller
              front_left_wheel_velocity_controller
              rear_left_wheel_velocity_controller
              rear_right_wheel_velocity_controller
              front_right_wheel_velocity_controller"/>

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
        respawn="false" output="screen">
    <remap from="/joint_states" to="$(arg robot_ns)/joint_states"/>
  </node>

  <node name="xdrive_kinematics_node" pkg="x_omni4wd_control" type="xdrive_kinematics_node"
        output="screen">
    <param name="robot_ns" value="$(arg robot_ns)" />
    <param name="wheel_radius" value="0.05" />
    <param name="odom_frame" value="odom" />
    <param name="base_frame" value="base_link" />
    <param name="publish_tf" value="true" />
    <param name="cmd_timeout" value="0.2" />
    <param name="sign_fl" value="1.0" />
    <param name="sign_rl" value="1.0" />
    <param name="sign_rr" value="1.0" />
    <param name="sign_fr" value="1.0" />
  </node>
</launch>
```

### File 5: `src/xdrive_kinematics_node.cpp` (Tóm tắt)

**Chức năng chính:**

1. **Inverse Kinematics** (cmd_vel → wheel speeds):
   ```cpp
   // v_i = [vx, vy]^T + ω * [-y_i, x_i]^T
   double vx_i = vx - omega * wheel_positions_[i].second;
   double vy_i = vy + omega * wheel_positions_[i].first;
   
   // d_i = [cos(alpha_i), sin(alpha_i)]
   double dx = std::cos(wheel_alpha_[i]);
   double dy = std::sin(wheel_alpha_[i]);
   
   // wheel_speed_i = (1/r) * d_i^T * v_i
   double wheel_speed = (1.0 / wheel_radius_) * (dx * vx_i + dy * vy_i);
   
   // Apply sign calibration
   wheel_velocities[i] = sign_per_wheel_[i] * wheel_speed;
   ```

2. **Forward Kinematics** (wheel feedback → odom):
   ```cpp
   // Solve: A * [vx, vy, ω]^T = b using least squares
   Eigen::MatrixXd A(4, 3);
   Eigen::VectorXd b(4);
   // Build matrix A and vector b from wheel velocities
   // Solve: x = (A^T * A)^(-1) * A^T * b
   ```

3. **Odometry Integration**:
   ```cpp
   // Integrate velocity to position
   x_ += (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
   y_ += (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
   yaw_ += omega * dt;
   ```

4. **Safety**: Stop wheels if no cmd_vel for `cmd_timeout` seconds

**Tham số ROS:**
- `wheel_radius`: 0.05 m
- `odom_frame`: "odom"
- `base_frame`: "base_link"
- `publish_tf`: true
- `cmd_timeout`: 0.2 s
- `sign_fl`, `sign_rl`, `sign_rr`, `sign_fr`: Sign calibration (default 1.0)

**Topics:**
- Sub: `/cmd_vel` (geometry_msgs/Twist)
- Sub: `/x_omni4wd/joint_states` (sensor_msgs/JointState)
- Pub: `/x_omni4wd/*/command` (std_msgs/Float64) - 4 topics
- Pub: `/odom` (nav_msgs/Odometry)
- TF: `odom` → `base_link`

---

## 🧪 (3) HƯỚNG DẪN TEST

### Test 1: Tiến thẳng (linear.x > 0)

**Lệnh:**
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.3, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Kỳ vọng:**
- Robot di chuyển về phía trước (theo hướng +x của base_link)
- Trong Gazebo: robot tiến về phía trước
- Odometry: `linear.x` tăng dần, `linear.y` ≈ 0

**Nếu ngược:**
- Robot lùi thay vì tiến → Đảo dấu tất cả `sign_*` trong launch file:
  ```xml
  <param name="sign_fl" value="-1.0" />
  <param name="sign_rl" value="-1.0" />
  <param name="sign_rr" value="-1.0" />
  <param name="sign_fr" value="-1.0" />
  ```

### Test 2: Dịch trái (linear.y > 0)

**Lệnh:**
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.3, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Kỳ vọng:**
- Robot dịch sang trái (theo hướng +y của base_link)
- Trong Gazebo: robot di chuyển sang trái mà không quay
- Odometry: `linear.y` tăng dần, `linear.x` ≈ 0

**Nếu ngược:**
- Robot dịch phải thay vì trái → Đảo dấu `sign_rl` và `sign_rr`:
  ```xml
  <param name="sign_rl" value="-1.0" />
  <param name="sign_rr" value="-1.0" />
  ```
  Hoặc đảo `sign_fl` và `sign_fr` tùy vào cấu hình.

### Test 3: Quay CCW (angular.z > 0)

**Lệnh:**
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 1.0}"
```

**Kỳ vọng:**
- Robot quay ngược chiều kim đồng hồ (CCW)
- Trong Gazebo: robot quay tại chỗ, không di chuyển
- Odometry: `angular.z` > 0, `yaw` tăng dần

**Nếu ngược:**
- Robot quay CW thay vì CCW → Đảo dấu `angular.z` trong cmd_vel hoặc điều chỉnh sign:
  ```xml
  <!-- Thử đảo dấu các bánh trái hoặc phải -->
  <param name="sign_fl" value="-1.0" />
  <param name="sign_rl" value="-1.0" />
  ```

### Cách chỉnh sign params nếu bị ngược

1. **Tạo file config riêng** (khuyến nghị):
   ```yaml
   # config/sign_calibration.yaml
   xdrive_kinematics_node:
     sign_fl: -1.0  # Đảo dấu nếu cần
     sign_rl: 1.0
     sign_rr: 1.0
     sign_fr: -1.0
   ```

2. **Load trong launch file**:
   ```xml
   <rosparam file="$(find x_omni4wd_control)/config/sign_calibration.yaml" command="load" />
   ```

3. **Hoặc sửa trực tiếp trong launch file**:
   ```xml
   <param name="sign_fl" value="-1.0" />
   ```

4. **Test lại sau mỗi lần thay đổi**

---

## ✅ KIỂM TRA BUILD

Package đã được build thành công:
```bash
[100%] Built target xdrive_kinematics_node
```

## 📍 ĐƯỜNG DẪN ĐẦY ĐỦ

- Package root: `/home/quyenanhpt/x_omni4wd_ws/src/x_omni4wd_control/`
- Executable: `/home/quyenanhpt/x_omni4wd_ws/devel/lib/x_omni4wd_control/xdrive_kinematics_node`
- Config: `/home/quyenanhpt/x_omni4wd_ws/src/x_omni4wd_control/config/x_omni4wd_control.yaml`
- Launch: `/home/quyenanhpt/x_omni4wd_ws/src/x_omni4wd_control/launch/x_omni4wd_ros_control.launch`

---

**Status:** ✅ Hoàn thành, đã build thành công, sẵn sàng test.

