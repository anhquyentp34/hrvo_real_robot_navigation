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
        self.prev_velocity = None
        self.velocity_pub = rospy.Publisher("/actor_velocity", Twist, queue_size=10)

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
            # dt = (current_time - self.prev_time).to_sec()
            # dx = current_position.x - self.prev_position.x
            # dy = current_position.y - self.prev_position.y
            # dz = current_position.z - self.prev_position.z

            # if dt > 0:
            #     velocity = Twist()
            #     velocity.linear = Vector3(dx/dt, dy/dt, dz/dt)
                
            #     if self.prev_velocity is not None:
            #         velocity_diff = Vector3(velocity.linear.x - self.prev_velocity.linear.x,
            #                                 velocity.linear.y - self.prev_velocity.linear.y,
            #                                 velocity.linear.z - self.prev_velocity.linear.z)
            #         diff_norm = (velocity_diff.x**2 + velocity_diff.y**2 + velocity_diff.z**2)**0.5
                    
            #         if diff_norm > 0.1:
            #             velocity = self.prev_velocity
                        
            #     self.velocity_pub.publish(velocity)
            #     self.prev_velocity = velocity

            velocity = Twist()
            velocity.linear = Vector3(current_position.x,current_position.y,current_position.z)
            self.velocity_pub.publish(velocity)
            self.prev_velocity = velocity

        self.prev_position = current_position
        self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node("actor_velocity_calculator")
    actor_name = rospy.get_param("~actor_name", "actor1")
    calc = ActorVelocityCalculator(actor_name)
    rospy.Subscriber("/gazebo/model_states", ModelStates, calc.model_states_callback)
    rospy.spin()
