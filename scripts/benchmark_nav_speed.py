#!/usr/bin/env python3
"""Đo vận tốc lệnh và odom sau khi gửi goal (so sánh profile HRVO)."""
import math
import sys
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def odom_speed(odom):
    v = odom.twist.twist.linear
    return math.hypot(v.x, v.y)


def main():
    rospy.init_node("benchmark_nav_speed", anonymous=True)
    gx = float(rospy.get_param("~goal_x", 0.0))
    gy = float(rospy.get_param("~goal_y", 4.0))
    wait_s = float(rospy.get_param("~wait_before_goal", 8.0))
    duration = float(rospy.get_param("~duration", 20.0))

    cmd_hrvo = Twist()
    cmd_vel = Twist()
    odom = [None]

    def cb_hrvo(m):
        nonlocal cmd_hrvo
        cmd_hrvo = m

    def cb_vel(m):
        nonlocal cmd_vel
        cmd_vel = m

    rospy.Subscriber("/cmd_vel_hrvo", Twist, cb_hrvo, queue_size=1)
    rospy.Subscriber("/cmd_vel", Twist, cb_vel, queue_size=1)
    rospy.Subscriber("/odom", Odometry, lambda m: odom.__setitem__(0, m), queue_size=1)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(120.0)):
        rospy.logerr("move_base timeout")
        sys.exit(1)
    rospy.sleep(wait_s)

    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "map"
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = gx
    g.target_pose.pose.position.y = gy
    g.target_pose.pose.orientation.w = 1.0
    client.send_goal(g)
    rospy.loginfo("Goal (%.1f, %.1f), đo %.0fs...", gx, gy, duration)

    r = rospy.Rate(10)
    n = int(duration * 10)
    max_ch = max_cv = max_od = 0.0
    sum_ch = sum_cv = sum_od = 0.0
    p0 = None
    for i in range(n):
        ch = abs(cmd_hrvo.linear.x)
        cv = abs(cmd_vel.linear.x)
        od = odom_speed(odom[0]) if odom[0] else 0.0
        if i == 0 and odom[0]:
            p0 = (odom[0].pose.pose.position.x, odom[0].pose.pose.position.y)
        max_ch = max(max_ch, ch)
        max_cv = max(max_cv, cv)
        max_od = max(max_od, od)
        sum_ch += ch
        sum_cv += cv
        sum_od += od
        r.sleep()

    p1 = None
    if odom[0]:
        p1 = (odom[0].pose.pose.position.x, odom[0].pose.pose.position.y)
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if p0 and p1 else 0.0

    rospy.loginfo("=== BENCHMARK ===")
    rospy.loginfo("  avg cmd_vel_hrvo.x: %.3f m/s", sum_ch / n)
    rospy.loginfo("  max cmd_vel_hrvo.x: %.3f m/s", max_ch)
    rospy.loginfo("  avg cmd_vel.x:      %.3f m/s", sum_cv / n)
    rospy.loginfo("  max cmd_vel.x:      %.3f m/s", max_cv)
    rospy.loginfo("  avg |odom| speed:   %.3f m/s", sum_od / n)
    rospy.loginfo("  max |odom| speed:   %.3f m/s", max_od)
    rospy.loginfo("  odom displacement:  %.2f m", dist)
    sys.exit(0)


if __name__ == "__main__":
    main()
