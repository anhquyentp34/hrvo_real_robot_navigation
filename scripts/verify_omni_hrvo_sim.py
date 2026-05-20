#!/usr/bin/env python3
"""Kiểm tra tự động x_omni4wd_hrvo_simulation."""
import math
import sys
import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hrvo_local_planner.msg import HRVOInput


def main():
    rospy.init_node("verify_omni_hrvo_sim", anonymous=True)
    timeout = float(rospy.get_param("~timeout", 120.0))
    goal_x = float(rospy.get_param("~goal_x", -7.0))
    goal_y = float(rospy.get_param("~goal_y", -4.0))

    results = []
    hrvo = [None]
    odom = [None]

    rospy.Subscriber("/hrvo/input", HRVOInput, lambda m: hrvo.__setitem__(0, m), queue_size=1)
    rospy.Subscriber("/odom", Odometry, lambda m: odom.__setitem__(0, m), queue_size=1)

    def wait_topic(name, min_hz=0.0, dur=3.0):
        t0 = rospy.Time.now()
        n0 = 0
        while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
            rospy.sleep(0.1)
            n0 += 1
        return True

    # 1. move_base
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(timeout)):
        results.append(("move_base", False, "không có action server"))
        print_report(results)
        sys.exit(1)
    results.append(("move_base", True, "OK"))

    # 2. /hrvo/input
    t0 = rospy.Time.now()
    while hrvo[0] is None and (rospy.Time.now() - t0).to_sec() < timeout:
        rospy.sleep(0.2)
    if hrvo[0] is None:
        results.append(("hrvo_input", False, "không nhận /hrvo/input"))
        print_report(results)
        sys.exit(2)
    static = sum(1 for a in hrvo[0].agents if a.social_state == "static_obstacle")
    dynamic = len(hrvo[0].agents) - static
    results.append(("hrvo_input", True, f"static={static} dynamic={dynamic}"))

    # 3. TF base_footprint
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    try:
        buf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(3.0))
        results.append(("tf_map_base", True, "OK"))
    except Exception as exc:
        results.append(("tf_map_base", False, str(exc)))

    # 4. joint_states / x_omni4wd
    try:
        rospy.wait_for_message("/x_omni4wd/joint_states", rospy.AnyMsg, timeout=10.0)
        results.append(("joint_states", True, "/x_omni4wd/joint_states"))
    except Exception:
        try:
            rospy.wait_for_message("/joint_states", rospy.AnyMsg, timeout=5.0)
            results.append(("joint_states", True, "/joint_states"))
        except Exception as exc:
            results.append(("joint_states", False, str(exc)))

    # 5. scan
    try:
        rospy.wait_for_message("/scan_filtered", rospy.AnyMsg, timeout=15.0)
        results.append(("scan", True, "/scan_filtered"))
    except Exception as exc:
        results.append(("scan", False, str(exc)))

    # 6. Gửi goal và theo dõi cmd_vel
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "map"
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = goal_x
    g.target_pose.pose.position.y = goal_y
    g.target_pose.pose.orientation.w = 1.0
    client.send_goal(g)

    cmd = [Twist()]

    def cb(m):
        cmd[0] = m

    rospy.Subscriber("/cmd_vel", Twist, cb, queue_size=1)
    rospy.sleep(15.0)

    max_v = 0.0
    dist = 0.0
    p0 = None
    if odom[0]:
        p0 = (odom[0].pose.pose.position.x, odom[0].pose.pose.position.y)
    for _ in range(30):
        max_v = max(max_v, math.hypot(cmd[0].linear.x, cmd[0].linear.y))
        rospy.sleep(0.5)
    if odom[0] and p0:
        p1 = (odom[0].pose.pose.position.x, odom[0].pose.pose.position.y)
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    moved = dist > 0.1 or max_v > 0.03
    results.append(("motion", moved, f"max|v|={max_v:.3f} dist={dist:.2f}m state={client.get_state()}"))

    print_report(results)
    failed = [r for r in results if not r[1]]
    sys.exit(0 if not failed else 3)


def print_report(results):
    print("\n=== KIỂM TRA OMNI HRVO ===")
    for name, ok, msg in results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name}: {msg}")
    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\nTổng: {len(results) - n_fail}/{len(results)} PASS")
    if n_fail == 0:
        print("KẾT LUẬN: PASS")
    else:
        print("KẾT LUẬN: CÓ LỖI")


if __name__ == "__main__":
    main()
