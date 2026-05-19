#!/usr/bin/env python3
"""
Spawn 4 tường khớp social_contexts (~30x30 m) qua /gazebo/spawn_sdf_model.
Dùng khi world Gazebo chỉ là pedsim_minimal_actor (ổn định) — tránh gzserver segfault
với world SDF nặng trên một số máy.
"""
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


# Độ dày tường (m): 0.2 nhìn từ trên rất khó thấy — tăng nhẹ để thấy trong GUI;
# vẫn khớp biên ~social_contexts (-0.5 … 29.5).
_WALL_THICK = "0.45"
WALLS = [
    ("pedsim_wall_1", "14.5 -0.5 1.4 0 0 0", "30.0 %s 2.8" % _WALL_THICK),
    ("pedsim_wall_2", "-0.5 14.5 1.4 0 0 1.57", "30.0 %s 2.8" % _WALL_THICK),
    ("pedsim_wall_3", "14.5 29.5 1.4 0 0 0", "30.0 %s 2.8" % _WALL_THICK),
    ("pedsim_wall_4", "29.5 14.5 1.4 0 0 1.57", "30.0 %s 2.8" % _WALL_THICK),
]


def parse_pose(pose_str):
    parts = [float(x) for x in pose_str.split()]
    x, y, z = parts[0], parts[1], parts[2]
    roll, pitch, yaw = parts[3], parts[4], parts[5]
    q = quaternion_from_euler(roll, pitch, yaw)
    return Pose(Point(x, y, z), Quaternion(q[0], q[1], q[2], q[3]))


def build_sdf(name, size_str):
    sx, sy, sz = size_str.split()
    return """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="%s">
    <static>true</static>
    <link name="link">
      <collision name="c">
        <geometry><box><size>%s %s %s</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>%s %s %s</size></box></geometry>
        <material>
          <ambient>0.45 0.42 0.38 1</ambient>
          <diffuse>0.72 0.68 0.62 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""" % (
        name,
        sx,
        sy,
        sz,
        sx,
        sy,
        sz,
    )


def main():
    rospy.init_node("spawn_social_context_walls")
    wait_after_ready = float(rospy.get_param("~wait_after_gazebo_ready", 2.5))
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=120.0)
    rospy.loginfo(
        "spawn_social_context_walls: chờ %.1fs để gzserver load world rồi spawn tường.",
        wait_after_ready,
    )
    rospy.sleep(wait_after_ready)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    for model_name, pose_str, size in WALLS:
        sdf = build_sdf(model_name, size)
        pose = parse_pose(pose_str)
        try:
            spawn(model_name, sdf, "", pose, "world")
            rospy.loginfo("Spawned wall model: %s", model_name)
        except rospy.ServiceException as e:
            rospy.logerr(
                "Spawn wall %s failed: %s — nếu đã spawn trước đó, xóa model trong Gazebo "
                "hoặc đổi tên / restart gzserver.",
                model_name,
                e,
            )
            return
    rospy.loginfo(
        "Xong 4 tường. Trong GUI: xoay camera (giữ chuột giữa) — nhìn từ trên xuống tường rất mỏng theo kích thước scene."
    )


if __name__ == "__main__":
    main()
