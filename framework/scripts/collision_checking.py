#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

def distance(point1, point2):
    return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

def predict_collision(teb_traj_points, object_position, object_velocity, time_horizon, distance_threshold):
    for teb_point in teb_traj_points:
        time_from_start = teb_point.time_from_start.to_sec()
        if time_from_start > time_horizon:
            break

        predicted_object_position = Point()
        predicted_object_position.x = object_position.x + object_velocity.x * time_from_start
        predicted_object_position.y = object_position.y + 0.9 * time_from_start
        predicted_object_position.z = object_position.z + object_velocity.z * time_from_start

      #  print(distance(teb_point.pose.position, predicted_object_position))
        print(distance_threshold)
        if distance(teb_point.pose.position, predicted_object_position) < distance_threshold:
            print("True")
            return True

    print("False")    
    return False

def teb_feedback_callback(feedback_msg):
#    print("I'm here")
    global object_position, object_velocity

    selected_traj = feedback_msg.trajectories[feedback_msg.selected_trajectory_idx]
    teb_traj_points = [point for point in selected_traj.trajectory]

    if predict_collision(teb_traj_points, object_position, object_velocity, time_horizon=5.0, distance_threshold=5):
        rospy.logwarn("Collision predicted between the robot and the object!")

def model_states_callback(msg):
   # print("I'm also here")
    global object_position, object_velocity, object_name

    object_index = -1
    for i, name in enumerate(msg.name):
        if name == object_name:
            object_index = i
            break

    if object_index == -1:
        rospy.logerr("Object not found in Gazebo model states")
        return

    object_position = msg.pose[object_index].position
    object_velocity = msg.twist[object_index].linear

rospy.init_node("collision_prediction")

object_name = rospy.get_param("~object_name", "actor1")
time_horizon = rospy.get_param("~time_horizon", 5.0)
distance_threshold = rospy.get_param("~distance_threshold", 5)

object_position = None
object_velocity = None

rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, teb_feedback_callback)

rospy.spin()
