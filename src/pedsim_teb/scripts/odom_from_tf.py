#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phát /odom từ TF odom -> base_footprint (khớp với simulate_diff_drive / static TF)."""
import math

import rospy
import tf2_ros
import tf.transformations as tft
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("odom_from_tf")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_frame = rospy.get_param("~base_frame", "base_footprint")
    rate_hz = rospy.get_param("~rate", 50.0)

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher("odom", Odometry, queue_size=10)

    rospy.loginfo("odom_from_tf: cho TF %s -> %s (pedbot phat sau khi khoi dong)...", odom_frame, base_frame)
    deadline = rospy.Time.now() + rospy.Duration(15.0)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
            break
        except Exception:
            rospy.sleep(0.05)

    last = None  # (stamp, x, y, yaw)
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn_throttle(5.0, "odom_from_tf: cho TF %s -> %s: %s", odom_frame, base_frame, ex)
            r.sleep()
            continue

        t = trans.header.stamp
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        msg = Odometry()
        msg.header.stamp = t
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = trans.transform.translation.z
        msg.pose.pose.orientation = q

        if last is not None:
            t0, x0, y0, yaw0 = last
            dt = (t - t0).to_sec()
            if dt > 1e-6:
                dx = x - x0
                dy = y - y0
                d_yaw = yaw - yaw0
                while d_yaw > math.pi:
                    d_yaw -= 2.0 * math.pi
                while d_yaw < -math.pi:
                    d_yaw += 2.0 * math.pi
                vx_w = dx / dt
                vy_w = dy / dt
                vx_b = math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
                msg.twist.twist.linear.x = vx_b
                msg.twist.twist.linear.y = 0.0
                msg.twist.twist.angular.z = d_yaw / dt
        # Đường chéo đầy đủ 6x6 (x,y,z,roll,pitch,yaw) — tránh RViz/TEB coi ma trận 3x3 vị trí = 0
        c = 1.0e-3
        for i in range(6):
            msg.pose.covariance[i * 6 + i] = c
            msg.twist.covariance[i * 6 + i] = c
        last = (t, x, y, yaw)
        pub.publish(msg)
        r.sleep()


if __name__ == "__main__":
    main()
