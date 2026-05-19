animated_markers
================

Animated mesh marker visualization plugin and accompanying ROS messages for Rviz

NOTE: This Rviz plugin has only been tested on ROS Hydro. It will not work on ROS Groovy.


Motivation
----------
Rviz currently does not play back animations contained in 3D meshes loaded from a ```visualization_msgs/Marker with type MESH_RESOURCE```.
This might be useful for displaying nice visualizations of e.g. walking persons in an environment.

Instead of modifying the source code of the original Rviz ```default_plugin```, the ```animated_marker_rviz_plugin``` replicates part of that code
and adds the required functionality specific to this purpose.

Example
-------
Run the following to publish example markers and see the visualization in Rviz.

```
roslaunch animated_marker_tutorial animated_walking_man.launch
```

Hiển thị actor Gazebo trong RViz (`actor_visualization_node`)
-------------------------------------------------------------
Node Python `animated_marker_tutorial/scripts/actor_visualization_node.py` đọc vị trí các model actor từ Gazebo (`gazebo_msgs/ModelStates`) và xuất `animated_marker_msgs/AnimatedMarkerArray` để plugin **AnimatedMarkerArray** trong RViz vẽ người đi bộ / đứng (mesh `.mesh` có skeleton hoặc mesh tĩnh `.dae`).

**Chạy cùng mô phỏng ICR2026 (khuyến nghị)** — trong workspace ICR2026, launch mô phỏng đã gọi sẵn node này khi bật tham số `enable_actor_visualization` (mặc định `true`):

```
roslaunch x_omni4wd_simulation ICR2026_simulation.launch
```

**Chạy riêng node** (Gazebo đã chạy và đang publish `/gazebo/model_states`):

```
rosrun animated_marker_tutorial actor_visualization_node.py
```

**RViz** — sau `source devel/setup.bash` (hoặc `setup.zsh`) của workspace:

1. Add → chọn display **AnimatedMarkerArray** (từ `animated_marker_rviz_plugin`).
2. Đặt topic marker (mặc định): `/actor_visualization`.
3. Nếu bật nhãn tên actor (`show_name_text`, mặc định bật): thêm display **MarkerArray** (plugin RViz chuẩn), topic `/actor_visualization_text`.

**Chủ đề ROS (mặc định)**

| Hướng | Topic | Kiểu message |
|-------|--------|----------------|
| Đăng ký (subscribe) | `/gazebo/model_states` | `gazebo_msgs/ModelStates` |
| Publish | `/actor_visualization` | `animated_marker_msgs/AnimatedMarkerArray` |
| Publish (tuỳ chọn) | `/actor_visualization_text` | `visualization_msgs/MarkerArray` |

**Tham số private (`~`) đáng chú ý** — có thể đặt trên node trong launch hoặc trong YAML dưới namespace node `actor_visualization_node`:

- `model_states_topic` — topic `ModelStates` (mặc định `/gazebo/model_states`).
- `output_topic` — tiền tố topic ra; marker chính = giá trị này, text = giá trị + `_text`.
- `frame_id` — thường `map` (khớp TF mô phỏng).
- `moving_human_mesh_resource` / `human_mesh_resource` — mesh đi bộ (OGRE `.mesh`); mặc định dùng `package://animated_marker_tutorial/meshes/animated_walking_man.mesh`.
- `standing_human_mesh_resource` — mesh đứng; mặc định `package://x_omni4wd_gazebo/models/person_standing/meshes/standing.dae` (cần gói `x_omni4wd_gazebo` trong workspace nếu giữ mặc định).
- `actor_name_regex` — chỉ model khớp regex mới được vẽ (mặc định `^actor\d+$`).
- `static_actor_name_regex` — nhóm actor được coi là “đứng yên” (mesh + không chạy animation đi bộ).
- `publish_rate`, `human_mesh_scale`, `standing_human_mesh_scale`, `show_name_text`, v.v. — xem docstring trong script.

**Phụ thuộc runtime** — `animated_marker_tutorial` cần `gazebo_msgs`, `animated_marker_msgs`, plugin RViz `animated_marker_rviz_plugin`; mesh đứng mặc định kéo thêm `x_omni4wd_gazebo`.

Usage
-----
The usage of this package is very similar to using ```visualization_msgs/MarkerArray and visualization_msgs/Marker```. Instead, just publish
```animated_marker_msgs/MarkerArray```. The only supported marker type is ```AnimatedMarker.MESH_RESOURCE```.

The animated_marker_rviz_plugin automatically registers itself with Rviz once the package is sourced (type eg. "source devel/setup.sh"
in your catkin workspace). You can then add an ```AnimatedMarkerArray``` display by clicking on the "Add display" button.

Supported animation formats
---------------------------
Currently, only OGRE *.mesh files (along with *.skeleton files) are supported for animation. These can be exported using e.g. 
Easy Ogre Exporter (http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Easy+Ogre+Exporter) from 3DS Max
or Blender Exporter (http://www.ogre3d.org/tikiwiki/Blender+Exporter) from the free Blender software.

Credits
-------
The animated_marker_rviz_plugin code is based upon the default_plugin in ros-visualization/rviz. Maintainer: quyenanh pt. The sample animated human mesh is a free low-poly mesh downloaded from mixamo.com, and the walking animation has been taken
from the Carnegie Mellon Motion Capture Database (or, more specifically, https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture/cmu-bvh-conversion).
