#!/usr/bin/env python
# license removed for brevity

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(msg):
    model_name = "actor1"  # Replace with the name of your object in Gazebo
    if model_name in msg.name:
        index = msg.name.index(model_name)
        pose = msg.pose[index]
        position = pose.position
        orientation = pose.orientation
        print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
        print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))

if __name__ == "__main__":
    rospy.init_node("get_pose")
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()