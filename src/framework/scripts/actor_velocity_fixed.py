#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header

class ActorVelocityCalculator:
    def __init__(self, actor_name):
        self.actor_name = actor_name
        self.prev_position = None
        self.prev_time = None
        self.velocity_pub = rospy.Publisher("/actor_velocity", Twist, queue_size=10)
        self.window_size = 5
        self.velocity_window = []

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
                velocity = Twist()
                velocity.linear = Vector3(dx/dt, dy/dt, dz/dt)
                
                # Apply moving average filter
                self.velocity_window.append(velocity)
                if len(self.velocity_window) > self.window_size:
                    self.velocity_window.pop(0)
                
                avg_velocity = Twist()
                for vel in self.velocity_window:
                    avg_velocity.linear.x += vel.linear.x
                    avg_velocity.linear.y += vel.linear.y
                    avg_velocity.linear.z += vel.linear.z
                
                avg_velocity.linear.x /= len(self.velocity_window)
                avg_velocity.linear.y /= len(self.velocity_window)
                avg_velocity.linear.z /= len(self.velocity_window)
                
                self.velocity_pub.publish(avg_velocity)

        self.prev_position = current_position
        self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node("actor_velocity_calculator")
    actor_name = rospy.get_param("~actor_name", "actor3")
    calc = ActorVelocityCalculator(actor_name)
    rospy.Subscriber("/gazebo/model_states", ModelStates, calc.model_states_callback)
    rospy.spin()
