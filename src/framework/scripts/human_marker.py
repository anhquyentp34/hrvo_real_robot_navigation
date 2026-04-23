#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker



def actor_pose_callback(msg):
    global object_position, object_velocity, object_name, object_marker_pub

    object_position = msg.pose.position

        # Create and publish the object marker
    object_marker = Marker()
    object_marker.header.frame_id = "map"
    object_marker.header.stamp = rospy.Time.now()
    object_marker.ns = "object"
    object_marker.id = 0
    object_marker.type = Marker.SPHERE
    object_marker.action = Marker.ADD
    object_marker.pose.position = object_position
    object_marker.pose.orientation.x = 0.0
    object_marker.pose.orientation.y = 0.0
    object_marker.pose.orientation.z = 0.0
    object_marker.pose.orientation.w = 1.0
    object_marker.scale.x = 1.0
    object_marker.scale.y = 1.0
    object_marker.scale.z = 1.0
    object_marker.color.r = 0.0
    object_marker.color.g = 1.0
    object_marker.color.b = 0.0
    object_marker.color.a = 1.0
    object_marker.lifetime = rospy.Duration(0)

    object_marker_pub.publish(object_marker)

rospy.init_node("add_human_marker")

rospy.Subscriber("/actor_pose", PoseStamped, actor_pose_callback)

object_marker_pub = rospy.Publisher("/object_marker", Marker, queue_size=10)

rospy.spin()