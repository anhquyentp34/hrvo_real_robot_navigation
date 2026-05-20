#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('map_to_odom_tf_publisher')

    # Create a TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

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

    rate = rospy.Rate(10)  # Set the frequency of the transform publication

    while not rospy.is_shutdown():
        static_transform.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(static_transform)
        rate.sleep()

if __name__ == '__main__':
    main()
