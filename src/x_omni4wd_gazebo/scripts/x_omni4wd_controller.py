#!/usr/bin/env python
"""
ROS Controller for x_omni4wd robot
4-wheel omni-directional robot with X-configuration
Uses ROS Control to control individual wheels
"""

import rospy
import numpy as np
import sys
import os

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

# Import kinematics module
# When installed via catkin, both scripts are in devel/lib/x_omni4wd_gazebo/
# So we can import directly after adding script_dir to path
try:
    import x_omni4wd_kinematics as kin
except ImportError:
    # Fallback: load directly from file
    import imp
    kin_file = os.path.join(script_dir, 'x_omni4wd_kinematics.py')
    if os.path.exists(kin_file):
        kin = imp.load_source('x_omni4wd_kinematics', kin_file)
    else:
        print("ERROR: Cannot find x_omni4wd_kinematics.py in %s" % script_dir)
        sys.exit(1)

# ROS Topics
CMD_VEL_TOPIC = '/cmd_vel'
MODEL_STATES_TOPIC = '/gazebo/model_states'
WHEEL_CMD_TOPICS = {
    'fl': '/x_omni4wd/front_left_wheel_controller/command',
    'rl': '/x_omni4wd/rear_left_wheel_controller/command',
    'rr': '/x_omni4wd/rear_right_wheel_controller/command',
    'fr': '/x_omni4wd/front_right_wheel_controller/command'
}

# Control parameters
CONTROL_FREQUENCY = 50  # Hz
ROBOT_NAME = 'x_omni4wd'  # Model name in Gazebo

# Current state
current_cmd_vel = Twist()
current_position = kin.Position(0.0, 0.0, 0.0)
cmd_vel_received = False
last_wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # FL, RL, RR, FR


def cmd_vel_callback(msg):
    """Callback for cmd_vel topic"""
    global current_cmd_vel, cmd_vel_received
    current_cmd_vel = msg
    cmd_vel_received = True


def model_states_callback(msg):
    """Callback for Gazebo model states"""
    global current_position
    
    try:
        # Find robot index
        robot_index = msg.name.index(ROBOT_NAME)
        
        # Get position
        pose = msg.pose[robot_index]
        current_position.x = pose.position.x
        current_position.y = pose.position.y
        
        # Get orientation (yaw)
        orientation = pose.orientation
        (_, _, current_position.yaw) = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
    except (ValueError, IndexError):
        rospy.logwarn_throttle(1.0, "Robot '%s' not found in Gazebo model states", ROBOT_NAME)


def main():
    """Main controller loop"""
    global current_cmd_vel, cmd_vel_received, last_wheel_velocities
    
    # Initialize ROS node
    rospy.init_node('x_omni4wd_controller', anonymous=True)
    
    # Publishers for wheel velocities
    pub_fl = rospy.Publisher(WHEEL_CMD_TOPICS['fl'], Float64, queue_size=10)
    pub_rl = rospy.Publisher(WHEEL_CMD_TOPICS['rl'], Float64, queue_size=10)
    pub_rr = rospy.Publisher(WHEEL_CMD_TOPICS['rr'], Float64, queue_size=10)
    pub_fr = rospy.Publisher(WHEEL_CMD_TOPICS['fr'], Float64, queue_size=10)
    
    # Subscribers
    rospy.Subscriber(CMD_VEL_TOPIC, Twist, cmd_vel_callback)
    rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, model_states_callback)
    
    rospy.loginfo("x_omni4wd_controller started")
    rospy.loginfo("Waiting for cmd_vel commands...")
    
    # Wait for publishers to be ready
    rospy.sleep(0.5)
    
    # Initialize wheels to zero (only once)
    pub_fl.publish(Float64(0.0))
    pub_rl.publish(Float64(0.0))
    pub_rr.publish(Float64(0.0))
    pub_fr.publish(Float64(0.0))
    
    rate = rospy.Rate(CONTROL_FREQUENCY)
    
    while not rospy.is_shutdown():
        if cmd_vel_received:
            # Calculate wheel velocities using inverse kinematics
            v_fl, v_rl, v_rr, v_fr = kin.inverse_kinematics(
                current_cmd_vel.linear.x,
                current_cmd_vel.linear.y,
                current_cmd_vel.angular.z
            )
            
            # Only publish if values changed significantly (avoid unnecessary publishes)
            threshold = 0.001  # rad/s
            if (abs(v_fl - last_wheel_velocities[0]) > threshold or
                abs(v_rl - last_wheel_velocities[1]) > threshold or
                abs(v_rr - last_wheel_velocities[2]) > threshold or
                abs(v_fr - last_wheel_velocities[3]) > threshold):
                
                # Publish wheel velocities (rad/s)
                pub_fl.publish(Float64(v_fl))
                pub_rl.publish(Float64(v_rl))
                pub_rr.publish(Float64(v_rr))
                pub_fr.publish(Float64(v_fr))
                
                # Update last values
                last_wheel_velocities = [v_fl, v_rl, v_rr, v_fr]
            
            # Debug output (throttled)
            rospy.logdebug_throttle(1.0, 
                "Cmd: v_x=%.2f, v_y=%.2f, ω_z=%.2f | "
                "Wheels: FL=%.2f, RL=%.2f, RR=%.2f, FR=%.2f",
                current_cmd_vel.linear.x,
                current_cmd_vel.linear.y,
                current_cmd_vel.angular.z,
                v_fl, v_rl, v_rr, v_fr
            )
        else:
            # Only stop wheels once when cmd_vel stops (not continuously)
            if any(abs(v) > 0.001 for v in last_wheel_velocities):
                pub_fl.publish(Float64(0.0))
                pub_rl.publish(Float64(0.0))
                pub_rr.publish(Float64(0.0))
                pub_fr.publish(Float64(0.0))
                last_wheel_velocities = [0.0, 0.0, 0.0, 0.0]
                rospy.logdebug("Stopped all wheels")
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
