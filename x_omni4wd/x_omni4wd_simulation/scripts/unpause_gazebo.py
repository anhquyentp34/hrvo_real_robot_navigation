#!/usr/bin/env python3
"""Unpause Gazebo sau khi controller ros_control đã load."""
import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node("gazebo_unpause", anonymous=True)
    rospy.loginfo("Đợi /gazebo/unpause_physics...")
    try:
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=60.0)
    except rospy.ROSException:
        rospy.logwarn("Không có dịch vụ unpause — bỏ qua (world có thể đã chạy)")
        return
    unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    try:
        unpause()
        rospy.loginfo("Đã unpause Gazebo.")
    except Exception as exc:
        rospy.logwarn("unpause_physics: %s", exc)


if __name__ == "__main__":
    main()
