#!/usr/bin/env python
# license removed for brevity

import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32

def create_obstacle():
    obstacle = ObstacleMsg()

    # Set the obstacle ID (choose any unique number)
    obstacle.id = 1

    # Define a polygon-shaped obstacle with 4 points
    point1 = Point32(1.0, 1.0, 0.0)
    point2 = Point32(2.0, 1.0, 0.0)
    point3 = Point32(2.0, 2.0, 0.0)
    point4 = Point32(1.0, 2.0, 0.0)

    obstacle.polygon.points = [point1, point2, point3, point4]
    # obstacle.polygon.points = [point1]
    obstacle.radius = 0.5


    return obstacle

def add_obstacle():
    rospy.init_node('custom_obstacle_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = 'map' # Change this to the appropriate frame

        obstacle = create_obstacle()
        obstacle_msg.obstacles.append(obstacle)

        pub.publish(obstacle_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        add_obstacle()
    except rospy.ROSInterruptException:
        pass
