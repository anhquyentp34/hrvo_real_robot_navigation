#!/usr/bin/env python3
"""Kiểm tra HRVO nhận vật cản tĩnh từ global costmap."""
import math
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hrvo_local_planner.msg import HRVOInput


def main():
    rospy.init_node("verify_static_obstacles", anonymous=True)
    timeout = float(rospy.get_param("~timeout", 90.0))
    goal_x = float(rospy.get_param("~goal_x", 2.0))
    goal_y = float(rospy.get_param("~goal_y", 0.0))
    min_static = int(rospy.get_param("~min_static_near_robot", 3))

    hrvo = [None]

    def cb(msg):
        hrvo[0] = msg

    rospy.Subscriber("/hrvo/input", HRVOInput, cb, queue_size=1)
    t0 = rospy.Time.now()
    while hrvo[0] is None and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            rospy.logerr("FAIL: không nhận /hrvo/input trong %.0fs", timeout)
            sys.exit(1)
        rospy.sleep(0.2)

    msg = hrvo[0]
    static = [a for a in msg.agents if a.social_state == "static_obstacle"]
    dynamic = [a for a in msg.agents if a.social_state != "static_obstacle"]
    rx = msg.robot_pose.position.x
    ry = msg.robot_pose.position.y

    def dist(a):
        return math.hypot(a.x - rx, a.y - ry)

    static_near = [a for a in static if dist(a) <= 8.0]
    rospy.loginfo("=== KIỂM TRA VẬT CẢN TĨNH HRVO ===")
    rospy.loginfo("  agents tĩnh (tổng): %d", len(static))
    rospy.loginfo("  agents động (pedsim): %d", len(dynamic))
    rospy.loginfo("  agents tĩnh trong 8m: %d", len(static_near))
    rospy.loginfo("  robot map (%.2f, %.2f)", rx, ry)

  # fusion topic param
    try:
        topic = rospy.get_param(
            "/hrvo_input_fusion_node/static_obstacle_map_topic", "?"
        )
        rospy.loginfo("  static map topic: %s", topic)
    except Exception:
        pass

    if len(static_near) < min_static:
        rospy.logwarn(
            "FAIL: quá ít vật cản tĩnh gần robot (%d < %d)",
            len(static_near),
            min_static,
        )
        sys.exit(2)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logwarn("move_base chưa sẵn sàng — bỏ qua test di chuyển")
        rospy.loginfo("PASS (static obstacles OK)")
        sys.exit(0)

    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "map"
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = goal_x
    g.target_pose.pose.position.y = goal_y
    g.target_pose.pose.orientation.w = 1.0
    client.send_goal(g)
    rospy.loginfo("Đã gửi goal (%.1f, %.1f), chờ 12s...", goal_x, goal_y)
    rospy.sleep(12.0)

    if hrvo[0] is not None:
        static2 = sum(
            1 for a in hrvo[0].agents if a.social_state == "static_obstacle"
        )
        rospy.loginfo("  agents tĩnh sau goal: %d", static2)

    state = client.get_state()
    rospy.loginfo("  move_base state: %d", state)
    rospy.loginfo("PASS: HRVO fusion có vật cản tĩnh từ costmap")
    sys.exit(0)


if __name__ == "__main__":
    main()
