# Ghi chep hoc thuat: Mo phong dieu huong robot trong moi truong dong nguoi

## 1) Muc tieu nghien cuu

Muc tieu cua dot thuc nghiem la xay dung va on dinh hoa pipeline mo phong robot vi sai trong moi truong dong nguoi (Pedsim + Gazebo + move_base), bao gom:
- Chay duoc world `airport_activities2_centered_singlemodel.world`.
- Dong bo toa do giua world centered va kich ban agent.
- Dam bao robot duoc spawn dung, co du lieu cam bien `/scan`, va nhan goal dieu huong.
- Dam bao agent trong Gazebo di chuyen theo du lieu tu `pedsim_simulator`.

## 2) Cau hinh va pham vi he thong

- Nen tang: ROS Noetic + Gazebo 11.
- Navigation stack: `move_base` + TEB local planner.
- Localization: map_server + AMCL (che do map-based duoc bat lai de on dinh he thong).
- Nguon du lieu nguoi di bo: `pedsim_simulator` -> `/pedsim_simulator/simulated_agents`.
- Cau hinh chinh:
  - Launch tong: `src/diff_simulation/launch/diff_bot_simulation.launch`
  - Robot model: `src/diff_description/urdf/diff_bot.urdf.xacro`
  - PointCloud->LaserScan: `src/pointcloud_to_scan/launch/pointcloud2scan.launch`
  - World centered: `src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/worlds/airport_activities2_centered_singlemodel.world`
  - Kich ban dong nguoi: `src/pedsim_ros_with_gazebo/pedsim_simulator/scenarios/airport_activities.xml` va `airport_activities2.xml`

## 3) Van de gap phai trong qua trinh thuc nghiem

### 3.1 Robot khong spawn duoc

Trieu chung:
- `Spawn service failed. Exiting.`
- `GetModelState: model [diff_bot] does not exist`

Chan doan:
- Co xung dot nhieu tien trinh Gazebo (`Unable to start server[bind: Address already in use]`).
- Service spawn bi anh huong boi cac phien Gazebo cu/treo.

Khac phuc:
- Don dep va khoi dong lai phien mo phong sach (kill cac tien trinh `gzserver/gzclient/roslaunch` cu).
- Dat lai pose spawn robot ve tam world (`x=0, y=0`) trong launch.

### 3.2 Khong co du lieu `/scan` on dinh cho AMCL/move_base

Trieu chung:
- Canh bao `No laser scan received`.
- `pointcloud_to_laserscan` khong phat scan dung ky vong.

Nguyen nhan:
- Topic point cloud dau vao trong launch khong khop voi topic thuc te cua Velodyne.
- `rosparam` trong `pointcloud2scan.launch` chua duoc substitute arg dung cach.
- `target_frame` de rong gay loi marshal `None`.

Khac phuc:
- Dong nhat `velodyne_cloud_in` ve `/velodyne_points`.
- Them `subst_value="true"` cho khoi `<rosparam>` trong `pointcloud2scan.launch`.
- Dat `target_frame` mac dinh la `base_footprint`.

### 3.3 Cay TF odom-base roi rac

Trieu chung:
- `Timed out waiting for transform from base_footprint to map...`
- Cac frame odom/base co luc khong lien thong day du khi khoi dong.

Khac phuc:
- Bo sung tham so plugin diff-drive trong URDF:
  - `odometrySource`
  - `publishOdomTF`
  - `publishTf`

### 3.4 Agent xuat hien nhung khong di chuyen

Trieu chung:
- `pedsim_simulator` van publish `/pedsim_simulator/simulated_agents` ~25 Hz.
- Agent trong Gazebo dung yen.

Nguyen nhan goc:
- World centered thieu plugin cap nhat pose actor tu Pedsim.
- File centered world khong co:
  - `<plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so"/>`

Khac phuc:
- Them lai `ActorPosesPlugin` vao `airport_activities2_centered_singlemodel.world`.

## 4) Cac thay doi ma nguon quan trong

1. `src/diff_simulation/launch/diff_bot_simulation.launch`
- Spawn pose robot ve tam world.
- Bat che do su dung map tinh cho localization.
- Dong nhat input pointcloud cho node chuyen doi scan.

2. `src/pointcloud_to_scan/launch/pointcloud2scan.launch`
- Them `subst_value="true"` trong `<rosparam>`.
- Dat `target_frame` mac dinh = `base_footprint`.

3. `src/diff_description/urdf/diff_bot.urdf.xacro`
- Bo sung tham so publish odom TF trong plugin `libgazebo_ros_diff_drive.so`.

4. `src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/worlds/airport_activities2_centered_singlemodel.world`
- Them plugin `ActorPosesPlugin` de dong bo pose agent.

5. Kich ban Pedsim centered
- Da tich chuyen toa do agent/waypoint/obstacle theo he quy chieu centered (dx=-30, dy=-15) trong cac file scenario lien quan.

## 5) Ket qua xac minh sau cung

- Robot `diff_bot` ton tai trong Gazebo (`/gazebo/get_model_state` success).
- Topic `/scan` phat du lieu hop le.
- Gui goal qua `/move_base_simple/goal` -> `cmd_vel` co gia tri khac 0 (robot bat dau dieu huong).
- Agent di chuyen trong Gazebo (kiem tra vi tri model theo thoi gian cho thay doi ro rang, `moved=True`).

## 6) Han che con lai va huong mo rong

- Con canh bao plugin:
  - `Failed to load plugin libcollision_map_creator.so`
  Van de nay khong con chan navigation/agent motion, nhung nen xu ly de log sach va de bao tri.

- Co canh bao `TF_REPEATED_DATA` tren mot so frame banh xe.
  Thuong khong gay dung he thong, nhung co the can toi uu de giam nhieu trong log.

## 7) Ket luan

Qua trinh debug da dua he thong tu trang thai loi nghiem trong (robot khong spawn, khong scan, agent dung yen) den trang thai hoat dong duoc de phuc vu thi nghiem:
- Robot spawn on dinh.
- Navigation stack nhan du lieu cam bien va phat lenh dieu khien.
- Dong nguoi trong Gazebo di chuyen theo Pedsim.

He thong hien san sang cho cac thi nghiem tiep theo ve hieu nang tranh nguoi, do an toan quy dao, va so sanh planner trong moi truong dong.
