#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

def publish_path():
    # Initialize the node
    rospy.init_node('path_publisher_node')
    
    # Create the publisher with topic "/cmd_path" and message type "Path"
    pub = rospy.Publisher('/cmd_path', Path, queue_size=1, latch=True)
    
    rate = rospy.Rate(1)  # Publish at 1 Hz
    
    while not rospy.is_shutdown():
        # Create the Path message
        path_msg = Path()
        
        # Set the header for the Path message
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        # Create waypoint 1: (10, 0)
        pose1 = PoseStamped()
        pose1.header.stamp = rospy.Time.now()
        pose1.header.frame_id = "map"
        pose1.pose.position = Point(x=10, y=0, z=0)
        # Orientation towards (-7, -7): angle -pi/2 (270 degrees)
        quat1 = Quaternion(*quaternion_from_euler(0, 0, -math.pi/2))
        pose1.pose.orientation = quat1
        
        # Create waypoint 2: (-10, 0)
        pose2 = PoseStamped()
        pose2.header.stamp = rospy.Time.now()
        pose2.header.frame_id = "map"
        pose2.pose.position = Point(x=-10, y=0, z=0)
        # Orientation towards (-7, 7): angle pi/2 (90 degrees)
        quat2 = Quaternion(*quaternion_from_euler(0, 0, math.pi/2))
        pose2.pose.orientation = quat2
        
        # Add waypoints to path
        path_msg.poses = [pose1, pose2]
        
        # Publish the Path message
        pub.publish(path_msg)
        # print('Published path:', path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
