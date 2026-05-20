#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt

def teb_poses_callback(msg):
    for pose in msg.poses:
        position = pose.position
        orientation = pose.orientation
        print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
        print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, 
                                                           orientation.z, orientation.w))

    x_data = []
    y_data = []
    
    for pose in msg.poses:
        position = pose.position
        x_data.append(position.x)
        y_data.append(position.y)

    plt.figure()
    plt.plot(x_data, y_data, '-o')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("TEB Trajectory")
    plt.show()

if __name__ == "__main__":
    rospy.init_node("teb_poses_listener")
    rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_poses", PoseArray, teb_poses_callback)
    rospy.spin()