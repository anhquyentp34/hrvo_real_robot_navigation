# diff_keyboard_teleop

Teleop ban phim toan he thong (pynput) cho robot vi sai.

## Dac diem

- Publish `geometry_msgs/Twist`.
- Dung `linear.x` va `angular.z` (khong strafe).
- Lam muot theo gia toc (tranh giat khi nhan/tha phim).

## Phim dieu khien

- `W/S`: tien/lui
- `A/D`: quay trai/phai
- `Space`: dung ngay
- `Esc`: thoat node
- Mui ten (tuy chon): len/xuong cho `vx`, trai/phai cho `wz`

## Cai dat phu thuoc

```bash
pip3 install --user pynput
```

## Chay nhanh

```bash
source ~/hrvo_real_robot_navigation/devel/setup.bash
roslaunch diff_keyboard_teleop diff_keyboard_teleop.launch
```

Gui qua gate trong `diff_control`:

```bash
roslaunch diff_keyboard_teleop diff_keyboard_teleop.launch cmd_vel_topic:=/cmd_vel_raw
```
