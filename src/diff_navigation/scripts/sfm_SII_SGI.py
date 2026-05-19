#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import os
from std_msgs.msg import Float32
from social_msgs.msg import SocialState

# =====================================================
# CSV PATH (AN TOÀN – CHẮC CHẮN GHI ĐƯỢC)
# =====================================================
CSV_PATH = "/home/hnam/nam_xp_ws/log/sfm_SII.csv"

# =====================================================
# SOCIAL PARAMETERS
# =====================================================
A_SII = 0.5
SIGMA_SII = 0.25

A_SGI = 1.0
SIGMA_MIN = 0.3

# =====================================================
# GLOBAL STATE
# =====================================================
robot_x = 0.0
robot_y = 0.0
agents = []   # [(x, y)]

# =====================================================
# UTILS
# =====================================================
def collision_index(x, y, x0, y0, A, sigma):
    d = math.hypot(x - x0, y - y0)
    return A * math.exp(-(d ** 2) / (2 * sigma ** 2))

# =====================================================
# SII
# =====================================================
def compute_sii(rx, ry):
    sii = 0.0
    for ax, ay in agents:
        sii = max(sii, collision_index(rx, ry, ax, ay, A_SII, SIGMA_SII))
    return sii

# =====================================================
# CALLBACK
# =====================================================
def social_state_cb(msg):
    global agents, robot_x, robot_y

    # -------- PEOPLE --------
    agents = []
    for p in msg.people.people:
        agents.append((p.position.position.x,
                       p.position.position.y))

    rx = robot_x
    ry = robot_y

    sii = compute_sii(rx, ry)

    # -------- SGI (GROUP THẬT) --------
    sgi = 0.0
    for g in msg.groups.groups:
        cx = g.position.position.x
        cy = g.position.position.y
        sigma = max(g.radius, SIGMA_MIN)
        sgi = max(sgi, collision_index(rx, ry, cx, cy, A_SGI, sigma))

    # -------- WRITE CSV --------
    # -------- WRITE CSV (FORMAT C++) --------
    try:
        with open(CSV_PATH, "a") as f:
            f.write("%.6f  %.6f  %.6f  %.6f\n" % (rx, ry, sii, sgi))
    except Exception as e:
        rospy.logerr(f"[SFM] CSV write failed: {e}")


    pub_sii.publish(sii)
    pub_sgi.publish(sgi)

# =====================================================
# ROBOT POSE FROM TF
# =====================================================
def update_robot_pose():
    global robot_x, robot_y
    try:
        tf = tf_buffer.lookup_transform(
            "map", "base_link",
            rospy.Time(0),
            rospy.Duration(0.1)
        )
        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
    except:
        pass

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    rospy.init_node("sfm_SII_SGI")

    # -------- PREPARE CSV FILE --------
    try:
        os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
        with open(CSV_PATH, "w") as f:
            pass
        rospy.loginfo(f"[SFM] CSV file created: {CSV_PATH}")
    except Exception as e:
        rospy.logerr(f"[SFM] Cannot create CSV file: {e}")

    pub_sii = rospy.Publisher(
        "/pedsim_simulator/Public_SII_sfm",
        Float32,
        queue_size=1
    )
    pub_sgi = rospy.Publisher(
        "/pedsim_simulator/Public_SGI_sfm",
        Float32,
        queue_size=1
    )

    rospy.Subscriber(
        "/social_state",
        SocialState,
        social_state_cb,
        queue_size=1
    )

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            update_robot_pose()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SFM] Node stopped cleanly.")

