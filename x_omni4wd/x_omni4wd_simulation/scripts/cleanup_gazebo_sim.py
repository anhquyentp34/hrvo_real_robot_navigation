#!/usr/bin/env python3
"""Xóa model Gazebo còn sót (tránh spawn trùng tên khi relaunch)."""
import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties


def main():
    rospy.init_node("gazebo_model_cleanup", anonymous=True)
    names = rospy.get_param("~model_names", "x_omni4wd")
    if isinstance(names, str):
        targets = [n.strip() for n in names.split(",") if n.strip()]
    else:
        targets = list(names)

    rospy.loginfo("Đợi dịch vụ Gazebo...")
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
        rospy.wait_for_service("/gazebo/delete_model", timeout=10.0)
    except rospy.ROSException:
        rospy.logwarn("Gazebo chưa sẵn sàng — bỏ qua cleanup (có thể là lần chạy đầu)")
        return

    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    try:
        props = get_world()
        existing = set(props.model_names)
    except Exception as exc:
        rospy.logwarn("Không đọc được world properties: %s", exc)
        return

    for name in targets:
        if name in existing:
            try:
                delete_model(name)
                rospy.loginfo("Đã xóa model Gazebo: %s", name)
            except Exception as exc:
                rospy.logwarn("Không xóa được %s: %s", name, exc)
        else:
            rospy.loginfo("Model %s không tồn tại — OK", name)


if __name__ == "__main__":
    main()
