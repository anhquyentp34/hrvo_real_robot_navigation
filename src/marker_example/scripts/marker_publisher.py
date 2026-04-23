#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def main():
    rospy.init_node('marker_publisher', anonymous=True)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    marker.id = 0
    marker.type = 3
    marker.action = Marker.ADD

    marker.pose.position.x = 1
    marker.pose.position.y = 1
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration()

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
