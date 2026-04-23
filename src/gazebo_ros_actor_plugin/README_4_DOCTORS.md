# Hướng dẫn sử dụng 4 DoctorFemaleWalk Actors

## Tổng quan
Package này đã được mở rộng để hỗ trợ 4 actor DoctorFemaleWalk cùng lúc, mỗi actor có thể được điều khiển độc lập với các đường đi khác nhau. **Các đường đi bắt đầu ngay từ vị trí khởi tạo của từng doctor, không đi về gốc tọa độ trước.**

## Các file đã tạo

### 1. World File
- **File**: `config/worlds/move_actor_4_doctors.world`
- **Mô tả**: Chứa 4 actor DoctorFemaleWalk ở 4 góc khác nhau của môi trường

### 2. Launch File
- **File**: `launch/sim_4_doctors.launch`
- **Mô tả**: File launch để khởi chạy simulation với 4 doctor actors và tự động chạy path publisher

### 3. Path Publisher Script
- **File**: `scripts/path_publisher_4_doctors.py`
- **Mô tả**: Script Python để điều khiển đường đi của 4 doctor actors (tự động chạy với launch file)

### 4. Plugin C++ (Đã sửa)
- **File**: `src/gazebo_ros_actor_command.cpp`
- **Mô tả**: Plugin C++ đã được sửa để không khởi tạo target pose mặc định về gốc tọa độ

### 5. Debug Script
- **File**: `scripts/debug_topics.py`
- **Mô tả**: Script để debug và kiểm tra các topic ROS

## Cách sử dụng

### Bước 1: Build và Source Workspace
```bash
# Build package
catkin_make

# Source workspace
source devel/setup.bash
```

### Bước 2: Khởi chạy Simulation (Tất cả trong một lệnh!)
```bash
# Khởi chạy simulation với 4 doctor actors và path publisher
roslaunch gazebo_ros_actor_plugin sim_4_doctors.launch
```

**Lưu ý**: Launch file này sẽ tự động:
- Khởi chạy Gazebo với world file chứa 4 doctor actors
- Chạy script `path_publisher_4_doctors.py` để điều khiển đường đi
- Không cần chạy thêm lệnh nào khác!

### Bước 3: Debug (Tùy chọn)
```bash
# Trong terminal mới, chạy script debug
rosrun gazebo_ros_actor_plugin debug_topics.py
```

## Chi tiết về 4 Doctor Actors

### Vị trí ban đầu và đường đi:
- **Doctor 1**: Góc tây bắc (-5, 5) - Đi theo hình vuông **bắt đầu từ vị trí khởi tạo**
- **Doctor 2**: Góc đông bắc (5, 5) - Đi theo hình tròn **bắt đầu từ vị trí khởi tạo**
- **Doctor 3**: Góc tây nam (-5, -5) - Đi theo đường thẳng **bắt đầu từ vị trí khởi tạo**
- **Doctor 4**: Góc đông nam (5, -5) - Đi theo hình tam giác **bắt đầu từ vị trí khởi tạo**

### Các topic ROS:
- **Doctor 1**: `/cmd_vel_1`, `/cmd_path_1`
- **Doctor 2**: `/cmd_vel_2`, `/cmd_path_2`
- **Doctor 3**: `/cmd_vel_3`, `/cmd_path_3`
- **Doctor 4**: `/cmd_vel_4`, `/cmd_path_4`

### Các đường đi được tạo (Bắt đầu từ vị trí khởi tạo):
1. **Hình vuông**: Doctor 1 đi theo hình vuông 6x6 đơn vị, bắt đầu từ (-5, 5)
2. **Hình tròn**: Doctor 2 đi theo đường tròn bán kính 3 đơn vị, bắt đầu từ (5, 5)
3. **Đường thẳng**: Doctor 3 đi qua lại theo đường thẳng 6 đơn vị, bắt đầu từ (-5, -5)
4. **Hình tam giác**: Doctor 4 đi theo hình tam giác đều, bắt đầu từ (5, -5)

### **Điểm cải tiến quan trọng:**
- **Trước đây**: DoctorFemaleWalk đi về gốc tọa độ (0,0) trước khi thực hiện đường đi
- **Bây giờ**: DoctorFemaleWalk bắt đầu đường đi ngay từ vị trí khởi tạo, tiết kiệm thời gian thí nghiệm

### **Các sửa đổi kỹ thuật:**
1. **Plugin C++**: Sửa hàm `Reset()` để không khởi tạo target pose mặc định về (0,0,0)
2. **Plugin C++**: Thêm kiểm tra `target_poses_.empty()` trong `OnUpdate()` để tránh lỗi
3. **Plugin C++**: Sửa hàm `PathCallback()` để clear target poses cũ và set target pose đầu tiên
4. **Script Python**: Sử dụng các hàm `*_from_start` để tạo đường đi từ vị trí khởi tạo

## Tùy chỉnh

### Thay đổi đường đi
Bạn có thể chỉnh sửa file `scripts/path_publisher_4_doctors.py` để:
- Thay đổi kích thước các hình
- Thay đổi vị trí trung tâm
- Tạo đường đi mới
- Sử dụng các hàm `*_from_start` để đảm bảo đường đi bắt đầu từ vị trí khởi tạo

### Thay đổi vị trí ban đầu
Chỉnh sửa tham số `pose` trong file `config/worlds/move_actor_4_doctors.world`:
```xml
<actor name="doctor1">
  <pose>-5 5 0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  ...
</actor>
```

### Thay đổi tham số chuyển động
Các tham số có thể điều chỉnh trong world file:
- `animation_factor`: Tốc độ animation
- `linear_velocity`: Tốc độ di chuyển
- `angular_velocity`: Tốc độ quay
- `linear_tolerance`: Độ chính xác vị trí
- `angular_tolerance`: Độ chính xác góc

### Tắt/bật path publisher
Nếu muốn chạy path publisher riêng biệt, bạn có thể:
1. Comment dòng node trong launch file:
```xml
<!-- <node name="path_publisher_4_doctors" pkg="gazebo_ros_actor_plugin" type="path_publisher_4_doctors.py" output="screen">
</node> -->
```

2. Chạy thủ công trong terminal riêng:
```bash
rosrun gazebo_ros_actor_plugin path_publisher_4_doctors.py
```

## Xử lý lỗi

### Lỗi thường gặp:
1. **Actor không di chuyển**: Kiểm tra topic đã được publish chưa
2. **Actor bị lật ngược**: Điều chỉnh `default_rotation` trong world file
3. **Collision**: Tăng khoảng cách giữa các actor
4. **Script không chạy**: Đảm bảo đã build lại package sau khi thêm script
5. **Actor đi về gốc tọa độ**: Đảm bảo đã build lại plugin C++ sau khi sửa đổi

### Debug:
```bash
# Kiểm tra các topic đang hoạt động
rostopic list

# Xem dữ liệu trên topic
rostopic echo /cmd_path_1

# Kiểm tra node đang chạy
rosnode list

# Kiểm tra log của path publisher
rosnode info /path_publisher_4_doctors

# Chạy script debug
rosrun gazebo_ros_actor_plugin debug_topics.py

# Kiểm tra plugin đã được load
gazebo --verbose
```

### Kiểm tra nhanh:
```bash
# Kiểm tra xem các topic có dữ liệu không
rostopic echo /cmd_path_1 -n 1
rostopic echo /cmd_path_2 -n 1
rostopic echo /cmd_path_3 -n 1
rostopic echo /cmd_path_4 -n 1
```

## Mở rộng

Để thêm nhiều actor hơn:
1. Thêm actor mới vào world file
2. Tạo topic mới cho actor
3. Cập nhật script path publisher (sử dụng hàm `*_from_start`)
4. Đảm bảo không có xung đột tên

## Tài liệu tham khảo
- [Gazebo Actor Tutorial](http://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot)
- [ROS Path Message](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) 