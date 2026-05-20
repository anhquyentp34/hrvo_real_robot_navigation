#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg

def tf_callback(msg):
    # Create a static TransformStamped message
    static_transform = geometry_msgs.msg.TransformStamped()

    # Set up the transform header and child/parent frame IDs
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = 'map'
    static_transform.child_frame_id = 'odom'

    # Set the translation and rotation (identity quaternion)
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    # Broadcast the transform from map to odom
    tf_broadcaster.sendTransform(static_transform)

rospy.init_node("map_to_odom_tf_publisher")
tf_broadcaster = tf2_ros.TransformBroadcaster()
rospy.Subscriber("/tf", TFMessage, tf_callback)
rospy.spin()
