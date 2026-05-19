# x_omni4wd_keyboard_teleop

Teleop bàn phím cho robot omni **x_omni4wd**: publish `geometry_msgs/Twist` đủ **vx, vy, ω** (ROS REP-103), có làm mượt theo gia tốc, bắt phím toàn hệ thống nhờ `pynput` (không cần focus cửa sổ terminal). Mọi luồng điều khiển bàn phím omni tập trung ở gói này (launch `teleop_omni_keyboard` trong bringup đã bỏ).

---

## Cài đặt

```bash
# Trong workspace
cd ~/costmapex_unified_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg x_omni4wd_keyboard_teleop
source devel/setup.bash

# Python
pip3 install --user pynput
```

---

## Chạy nhanh

**Chỉ teleop** (đã có `roscore` và bridge `/cmd_vel` — ví dụ bringup/simulation chạy riêng):

```bash
roslaunch x_omni4wd_keyboard_teleop x_omni4wd_keyboard_teleop.launch launch_robot:=false
```

**Teleop + bringup robot thật** (Arduino serial mặc định, không RViz/rqt steering):

```bash
roslaunch x_omni4wd_keyboard_teleop x_omni4wd_keyboard_teleop.launch launch_robot:=true
```

---

## Phím điều khiển

| Phím | Tác dụng |
|------|----------|
| **W** / **S** | Tiến / lùi (`linear.x`) |
| **Q** / **E** | Trượt trái / phải (`linear.y`, REP-103: +y trái) |
| **A** / **D** | Quay CCW / CW (`angular.z`) |
| **Space** | Dừng (xóa mục tiêu vận tốc) |
| **Esc** | Thoát node |

Nếu `use_arrow_keys:=true` (mặc định): **↑↓** = tiến/lùi, **←→** = quay (song song với **W/S** và **A/D**).

---

## Tham số launch / node (rút gọn)

| Tham số | Mặc định | Ý nghĩa |
|---------|----------|---------|
| `cmd_vel_topic` | `/cmd_vel` | Topic publish `Twist` |
| `launch_robot` | `true` | Có include `x_omni4wd_bringup.launch` hay không |
| `max_vx`, `max_vy`, `max_wz` | 0.5, 0.5, 1.0 | Giới hạn độ lớn \|v\|, \|ω\| |
| `accel_linear` | 1.5 | Gia tốc tuyến tính (m/s²), làm mượt vx/vy |
| `accel_angular` | 2.5 | Gia tốc góc (rad/s²), làm mượt ω |
| `publish_rate` | 50 | Tần số publish (Hz) |
| `use_arrow_keys` | true | Bật mũi tên |
| `invert_angular_z` | false | Đảo dấu `angular.z` **chỉ** tại node teleop (nếu quay ngược so với mong muốn) |

Ví dụ đảo chiều quay, bật LiDAR bringup:

```bash
roslaunch x_omni4wd_keyboard_teleop x_omni4wd_keyboard_teleop.launch \
  launch_robot:=true use_lidar:=true invert_angular_z:=true
```

Tham số `vx_forward`, `vx_backward`, `vy_*`, `wz_*` có thể set thêm trong launch hoặc `rosparam` nếu cần tốc độ từng chiều khác nhau (xem `x_omni4wd_keyboard_teleop.launch`).

---

## An toàn

- Giữ **Space** hoặc thả hết phím để robot giảm dần tốc độ (theo `accel_*`), rồi dừng.
- **Esc** thoát node và gửi vài lệnh `Twist` không (an toàn khi tắt).
- Trên robot thật: kiểm tra `/dev/robot_arduino`, baud, và không chạy trùng nhiều nguồn `cmd_vel` không điều phối.

---

## Gói phụ thuộc ROS

- `rospy`, `geometry_msgs`
- Khi `launch_robot:=true`: cần `x_omni4wd_bringup` (và các gói bringup kéo theo).
