#!/usr/bin/env python3

import rospy
import numpy as np
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from social_msgs.msg import SocialState, SocialPeople, SocialPerson

class ActorController:
    def __init__(self):
        rospy.init_node('actor_controller', anonymous=True)
        
        # Parameters
        self.actor_names = rospy.get_param('~actor_names', 'actor30,actor31,actor32,actor33').split(',')
        self.update_rate = rospy.get_param('~update_rate', 10)
        self.enable_movement = rospy.get_param('~enable_movement', True)
        self.movement_pattern = rospy.get_param('~movement_pattern', 'circular')
        
        # Movement parameters
        self.circle_radius = 3.0
        self.circle_center = [0.0, 0.0]
        self.angular_velocity = 0.5  # rad/s
        self.linear_velocity = 0.5   # m/s
        
        # Publishers
        self.cmd_vel_pubs = {}
        for actor_name in self.actor_names:
            topic_name = f"/{actor_name}/cmd_vel"
            self.cmd_vel_pubs[actor_name] = rospy.Publisher(topic_name, Twist, queue_size=10)
        
        # Subscribers
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # State variables
        self.actor_positions = {}
        self.actor_velocities = {}
        self.time_start = rospy.Time.now()
        
        # Timer for movement updates
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_movement)
        
        rospy.loginfo(f"ActorController initialized with actors: {self.actor_names}")
        rospy.loginfo(f"Movement pattern: {self.movement_pattern}")
        
    def model_states_callback(self, msg):
        """Callback to get actor positions from Gazebo"""
        for i, name in enumerate(msg.name):
            if name in self.actor_names:
                self.actor_positions[name] = msg.pose[i]
                self.actor_velocities[name] = msg.twist[i]
    
    def calculate_circular_movement(self, actor_name, time_elapsed):
        """Calculate circular movement for an actor"""
        # Different starting angles for each actor
        actor_index = self.actor_names.index(actor_name)
        start_angle = actor_index * (2 * math.pi / len(self.actor_names))
        
        # Calculate current angle
        current_angle = start_angle + self.angular_velocity * time_elapsed
        
        # Calculate position
        x = self.circle_center[0] + self.circle_radius * math.cos(current_angle)
        y = self.circle_center[1] + self.circle_radius * math.sin(current_angle)
        
        # Calculate velocity
        vx = -self.circle_radius * self.angular_velocity * math.sin(current_angle)
        vy = self.circle_radius * self.angular_velocity * math.cos(current_angle)
        
        return x, y, vx, vy, current_angle
    
    def calculate_linear_movement(self, actor_name, time_elapsed):
        """Calculate linear movement for an actor"""
        # Different starting positions for each actor
        actor_index = self.actor_names.index(actor_name)
        start_x = actor_index * 2.0 - 3.0  # Spread actors horizontally
        
        # Calculate position
        x = start_x + self.linear_velocity * time_elapsed
        y = 0.0
        
        # Calculate velocity
        vx = self.linear_velocity
        vy = 0.0
        
        return x, y, vx, vy, 0.0
    
    def calculate_random_movement(self, actor_name, time_elapsed):
        """Calculate random movement for an actor"""
        # Use time and actor index to generate deterministic "random" movement
        actor_index = self.actor_names.index(actor_name)
        seed = int(time_elapsed * 10) + actor_index * 1000
        
        np.random.seed(seed)
        
        # Random position within bounds
        x = np.random.uniform(-5.0, 5.0)
        y = np.random.uniform(-5.0, 5.0)
        
        # Random velocity
        vx = np.random.uniform(-1.0, 1.0)
        vy = np.random.uniform(-1.0, 1.0)
        
        return x, y, vx, vy, 0.0
    
    def update_movement(self, event):
        """Update actor movement based on pattern"""
        if not self.enable_movement:
            return
            
        time_elapsed = (rospy.Time.now() - self.time_start).to_sec()
        
        for actor_name in self.actor_names:
            if self.movement_pattern == 'circular':
                x, y, vx, vy, angle = self.calculate_circular_movement(actor_name, time_elapsed)
            elif self.movement_pattern == 'linear':
                x, y, vx, vy, angle = self.calculate_linear_movement(actor_name, time_elapsed)
            elif self.movement_pattern == 'random':
                x, y, vx, vy, angle = self.calculate_random_movement(actor_name, time_elapsed)
            else:
                # Default to stationary
                x, y, vx, vy, angle = 0.0, 0.0, 0.0, 0.0, 0.0
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.angular_velocity if self.movement_pattern == 'circular' else 0.0
            
            # Publish velocity command
            if actor_name in self.cmd_vel_pubs:
                self.cmd_vel_pubs[actor_name].publish(twist)
                
                # Log movement for debugging
                if rospy.get_param('~enable_debug', False):
                    rospy.loginfo(f"{actor_name}: pos=({x:.2f}, {y:.2f}), vel=({vx:.2f}, {vy:.2f}), ang_vel={twist.angular.z:.2f}")
    
    def print_actor_info(self):
        """Print current actor information"""
        rospy.loginfo("=== Actor Information ===")
        for actor_name in self.actor_names:
            if actor_name in self.actor_positions:
                pos = self.actor_positions[actor_name]
                vel = self.actor_velocities[actor_name]
                rospy.loginfo(f"{actor_name}:")
                rospy.loginfo(f"  Position: ({pos.position.x:.2f}, {pos.position.y:.2f}, {pos.position.z:.2f})")
                rospy.loginfo(f"  Linear Velocity: ({vel.linear.x:.2f}, {vel.linear.y:.2f}, {vel.linear.z:.2f})")
                rospy.loginfo(f"  Angular Velocity: ({vel.angular.x:.2f}, {vel.angular.y:.2f}, {vel.angular.z:.2f})")
            else:
                rospy.loginfo(f"{actor_name}: Not found in model states")
    
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(1)  # 1 Hz for info updates
        
        while not rospy.is_shutdown():
            if rospy.get_param('~enable_debug', False):
                self.print_actor_info()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = ActorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 