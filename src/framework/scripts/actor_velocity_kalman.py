#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header
from pykalman import KalmanFilter
import numpy as np

class ActorVelocityCalculator:
    def __init__(self, actor_name):
        self.actor_name = actor_name
        self.prev_position = None
        self.prev_time = None
        self.velocity_pub = rospy.Publisher("/actor_velocity", Twist, queue_size=10)

        # Initialize Kalman filter
        self.kf = KalmanFilter(transition_matrices=np.eye(3),
                               observation_matrices=np.eye(3),
                               initial_state_mean=np.zeros(3),
                               initial_state_covariance=np.eye(3),
                               transition_covariance=0.01 * np.eye(3),
                               observation_covariance=np.eye(3))

    def model_states_callback(self, msg):
        actor_index = -1
        for i, name in enumerate(msg.name):
            if name == self.actor_name:
                actor_index = i
                break

        if actor_index == -1:
            rospy.logerr("Actor not found in Gazebo model states")
            return

        current_position = msg.pose[actor_index].position
        current_time = rospy.Time.now()

        if self.prev_position is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).to_sec()
            dx = current_position.x - self.prev_position.x
            dy = current_position.y - self.prev_position.y
            dz = current_position.z - self.prev_position.z

            if dt > 0:
                raw_velocity = np.array([dx/dt, dy/dt, dz/dt])

                # Apply Kalman filter
                filtered_state_means, _ = self.kf.filter_update(self.kf.initial_state_mean,
                                                                self.kf.initial_state_covariance,
                                                                raw_velocity)

                velocity = Twist()
                velocity.linear = Vector3(filtered_state_means[0], filtered_state_means[1], filtered_state_means[2])
                self.velocity_pub.publish(velocity)

        self.prev_position = current_position
        self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node("actor_velocity_calculator")
    actor_name = rospy.get_param("~actor_name", "actor1")
    calc = ActorVelocityCalculator(actor_name)
    rospy.Subscriber("/gazebo/model_states", ModelStates, calc.model_states_callback)
    rospy.spin()
