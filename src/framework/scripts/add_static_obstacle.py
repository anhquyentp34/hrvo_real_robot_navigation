#!/usr/bin/env python
# license removed for brevity
import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32

def static_obstacle():
    obstacle_array = ObstacleArrayMsg()
    circle_obstacle = ObstacleMsg()
    # circle_obstacle.id = 2
    # circle_obstacle.radius = 0.2  # Set the radius of the circle
    circle_obstacle.polygon.points.append(Point32(2, 2, 0))  # Set the center of the circle
    circle_obstacle.polygon.points.append(Point32(3, 2, 0))
    circle_obstacle.polygon.points.append(Point32(3, 3, 0))
    circle_obstacle.polygon.points.append(Point32(2, 3, 0))
    circle_obstacle.polygon.points.append(Point32(2.2, 2.2, 0))
    circle_obstacle.polygon.points.append(Point32(2.8, 2.2, 0))
    circle_obstacle.polygon.points.append(Point32(2.8, 2.8, 0))
    circle_obstacle.polygon.points.append(Point32(2.2, 2.8, 0))
    

    obstacle_array.obstacles.append(circle_obstacle)
    pub = rospy.Publisher("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=10)
    rospy.init_node('add_static_obstacle', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(obstacle_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        static_obstacle()
    except rospy.ROSInterruptException:
        pass