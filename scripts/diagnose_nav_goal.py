#!/usr/bin/env python3
"""Gửi goal move_base và theo dõi cmd_vel / odom (dùng cho chẩn đoán tự động)."""
import math
import sys
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class NavDiag:
    def __init__(self):
        self.cmd_hrvo = Twist()
        self.cmd_vel = Twist()
        self.odom = None
        rospy.Subscriber("/cmd_vel_hrvo", Twist, self._cb_hrvo, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self._cb_vel, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._cb_odom, queue_size=1)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def _cb_hrvo(self, msg):
        self.cmd_hrvo = msg

    def _cb_vel(self, msg):
        self.cmd_vel = msg

    def _cb_odom(self, msg):
        self.odom = msg

    def odom_xy(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y)

    def wait_move_base(self, timeout=120.0):
        rospy.loginfo("Đợi move_base...")
        if not self.client.wait_for_server(rospy.Duration(timeout)):
            rospy.logerr("move_base không sẵn sàng sau %.0fs", timeout)
            return False
        rospy.loginfo("move_base OK")
        return True

    def send_goal(self, x, y, yaw=0.0):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.send_goal(g)
        rospy.loginfo("Đã gửi goal map (%.2f, %.2f)", x, y)

    def sample(self, duration=25.0, rate_hz=2.0):
        r = rospy.Rate(rate_hz)
        n = int(duration * rate_hz)
        p0 = self.odom_xy()
        max_vx_hrvo = 0.0
        max_vx_cmd = 0.0
        nonzero_hrvo = 0
        nonzero_cmd = 0
        for _ in range(n):
            if abs(self.cmd_hrvo.linear.x) > 1e-4 or abs(self.cmd_hrvo.angular.z) > 1e-4:
                nonzero_hrvo += 1
            if abs(self.cmd_vel.linear.x) > 1e-4 or abs(self.cmd_vel.angular.z) > 1e-4:
                nonzero_cmd += 1
            max_vx_hrvo = max(max_vx_hrvo, abs(self.cmd_hrvo.linear.x))
            max_vx_cmd = max(max_vx_cmd, abs(self.cmd_vel.linear.x))
            r.sleep()
        p1 = self.odom_xy()
        dist = 0.0
        if p0 and p1:
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        state = self.client.get_state()
        return {
            "max_vx_hrvo": max_vx_hrvo,
            "max_vx_cmd": max_vx_cmd,
            "nonzero_hrvo": nonzero_hrvo,
            "nonzero_cmd": nonzero_cmd,
            "odom_dist_m": dist,
            "move_base_state": state,
            "p0": p0,
            "p1": p1,
        }


def main():
    rospy.init_node("diagnose_nav_goal", anonymous=True)
    x = float(rospy.get_param("~goal_x", 0.0))
    y = float(rospy.get_param("~goal_y", 2.0))
    wait_s = float(rospy.get_param("~wait_before_goal", 8.0))
    diag = NavDiag()
    if not diag.wait_move_base():
        sys.exit(1)
    rospy.sleep(wait_s)
    p_start = diag.odom_xy()
    rospy.loginfo("Odom ban đầu: %s", p_start)
    diag.send_goal(x, y)
    rospy.sleep(3.0)
    rep = diag.sample(duration=25.0)
    rospy.loginfo("=== KẾT QUẢ CHẨN ĐOÁN ===")
    for k, v in rep.items():
        rospy.loginfo("  %s: %s", k, v)
    ok = rep["odom_dist_m"] > 0.15 or rep["max_vx_cmd"] > 0.05
    if ok:
        rospy.loginfo("PASS: robot có vận tốc hoặc đã dịch chuyển")
        sys.exit(0)
    rospy.logwarn("FAIL: robot gần như đứng yên — kiểm tra planner / adapter / goal")
    sys.exit(2)


if __name__ == "__main__":
    main()
