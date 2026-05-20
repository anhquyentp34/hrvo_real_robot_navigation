#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
from geometry_msgs.msg import PoseStamped

def teb_feedback_callback(feedback_msg):
    selected_traj = feedback_msg.trajectories[feedback_msg.selected_trajectory_idx]
    trajectory_points = selected_traj.trajectory

    # Extract pose, velocity, acceleration, and time_from_start
    poses = [point.pose for point in trajectory_points]
    velocities = [point.velocity for point in trajectory_points]
    accelerations = [point.acceleration for point in trajectory_points]
    time_from_start = [point.time_from_start for point in trajectory_points]

    print("Poses:", poses)
  #  print("Velocities:", velocities)
  #  print("Accelerations:", accelerations)
    print("Time from start:", time_from_start)

if __name__ == "__main__":
    rospy.init_node("teb_feedback_listener")
    rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, teb_feedback_callback)
    rospy.spin()