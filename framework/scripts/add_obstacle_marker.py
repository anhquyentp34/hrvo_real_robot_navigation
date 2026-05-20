#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def create_cylinder_marker():
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "cylinder_marker"
    marker.id = 0

    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position.x = 2
    marker.pose.position.y = 2
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.color.r = 0.5
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1

    marker.lifetime = rospy.Duration()

    return marker

if __name__ == "__main__":
    rospy.init_node("cylinder_marker_node")

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        cylinder_marker = create_cylinder_marker()
        marker_pub.publish(cylinder_marker)
        rate.sleep()
