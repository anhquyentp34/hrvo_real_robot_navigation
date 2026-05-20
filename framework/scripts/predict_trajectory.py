#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def model_states_callback(msg):
    global object_name
    object_index = -1

    for i, name in enumerate(msg.name):
        if name == object_name:
            object_index = i
            break

    if object_index == -1:
        rospy.logerr("Object not found in Gazebo model states")
        return

    current_position = msg.pose[object_index].position
    current_velocity = msg.twist[object_index].linear

    predicted_path = Path()
    predicted_path.header.stamp = rospy.Time.now()
    predicted_path.header.frame_id = "odom"

    for i in range(1, num_future_steps + 1):
        t = i * prediction_time_step
        predicted_position = PoseStamped()
        predicted_position.header.stamp = rospy.Time.now()
        predicted_position.header.frame_id = "odom"
        predicted_position.pose.position.x = current_position.x + current_velocity.x * t
        predicted_position.pose.position.y = current_position.y + 0.9 * t
        predicted_position.pose.position.z = current_position.z + current_velocity.z * t

        predicted_path.poses.append(predicted_position)

        print(predicted_path)

    trajectory_pub.publish(predicted_path)

rospy.init_node("object_trajectory_predictor")
object_name = rospy.get_param("~object_name", "actor1")  # Set your object's name in Gazebo
num_future_steps = rospy.get_param("~num_future_steps", 10)
prediction_time_step = rospy.get_param("~prediction_time_step", 0.5)  # Time step between predictions in seconds

rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
trajectory_pub = rospy.Publisher("/predicted_trajectory", Path, queue_size=1)

rospy.spin()