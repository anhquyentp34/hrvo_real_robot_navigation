#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def send_2d_nav_goal(x, y, theta):
    rospy.init_node('send_2d_nav_goal', anonymous=True)

    # Create a publisher to send the goal to the move_base node
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # Wait until the publisher is connected to the move_base node
    while goal_pub.get_num_connections() < 1:
        rospy.sleep(1)

    # Create a PoseStamped message
    goal = PoseStamped()
    goal.header.frame_id = 'map'  # Set the reference frame
    goal.header.stamp = rospy.Time.now()

    # Set the goal position (x, y) and orientation (theta)
    goal.pose.position.x = x
    goal.pose.position.y = y
    quat = quaternion_from_euler(0, 0, theta)
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    # Publish the goal
    goal_pub.publish(goal)
    rospy.loginfo("Sent 2D Nav Goal (x: {}, y: {}, theta: {})".format(x, y, theta))

if __name__ == '__main__':
    try:
        x = 6.0  # Set your desired x coordinate
        y = -5.0  # Set your desired y coordinate
        theta = -3.14159/4  # Set your desired orientation (in radians)
        # x = 6.0  # Set your desired x coordinate
        # y = 0  # Set your desired y coordinate
        # theta = 0 # Set your desired orientation (in radians)
        send_2d_nav_goal(x, y, theta)
    except rospy.ROSInterruptException:
        pass
