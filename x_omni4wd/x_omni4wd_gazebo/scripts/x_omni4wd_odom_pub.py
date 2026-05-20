#!/usr/bin/env python
"""
Odometry Publisher for x_omni4wd robot
Publishes odometry from wheel velocities using forward kinematics
Based on quicksilver odom_pub.py
"""

import rospy
import numpy as np
import sys
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist, Quaternion
import tf

# Add scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Import kinematics module
# When installed via catkin, both scripts are in devel/lib/x_omni4wd_gazebo/
# So we can import directly after adding script_dir to path
try:
    import x_omni4wd_kinematics as kin
except ImportError:
    # Fallback: load directly from file
    try:
        import imp
        kin_file = os.path.join(script_dir, 'x_omni4wd_kinematics.py')
        if os.path.exists(kin_file):
            kin = imp.load_source('x_omni4wd_kinematics', kin_file)
        else:
            rospy.logerr("ERROR: Cannot find x_omni4wd_kinematics.py in %s" % script_dir)
            sys.exit(1)
    except Exception as e:
        rospy.logerr("ERROR: Cannot import x_omni4wd_kinematics: %s" % str(e))
        sys.exit(1)

# ROS Topics
ODOM_TOPIC = '/odom'
JOINT_STATES_TOPIC = '/x_omni4wd/joint_states'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

# Robot parameters
WHEEL_RADIUS = 0.05  # m
WHEEL_SEPARATION_X = 0.2  # m
WHEEL_SEPARATION_Y = 0.2  # m

# Current state
current_wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # [FL, RL, RR, FR]
last_time = None
x = 0.0
y = 0.0
yaw = 0.0


def joint_states_callback(msg):
    """Callback for joint states"""
    global current_wheel_velocities
    
    try:
        # Find wheel joint indices - Thứ tự: (1,2,3,4) = (FL,RL,RR,FR)
        fl_idx = msg.name.index('base_link_front_left_wheel_joint')
        rl_idx = msg.name.index('base_link_rear_left_wheel_joint')
        rr_idx = msg.name.index('base_link_rear_right_wheel_joint')
        fr_idx = msg.name.index('base_link_front_right_wheel_joint')
        
        # Get wheel velocities (rad/s)
        current_wheel_velocities[0] = msg.velocity[fl_idx]  # FL
        current_wheel_velocities[1] = msg.velocity[rl_idx]  # RL
        current_wheel_velocities[2] = msg.velocity[rr_idx]  # RR
        current_wheel_velocities[3] = msg.velocity[fr_idx]  # FR
    except (ValueError, IndexError) as e:
        rospy.logwarn_throttle(1.0, "Cannot find wheel joints in joint_states: %s", str(e))


def main():
    """Main odometry publisher loop"""
    global last_time, x, y, yaw
    
    # Initialize ROS node
    rospy.init_node('x_omni4wd_odom_pub', anonymous=True)
    
    # Publishers
    odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=50)
    tf_broadcaster = tf.TransformBroadcaster()
    
    # Subscriber
    rospy.Subscriber(JOINT_STATES_TOPIC, JointState, joint_states_callback)
    
    rospy.loginfo("x_omni4wd_odom_pub started")
    rospy.loginfo("Publishing odometry from wheel velocities")
    
    rate = rospy.Rate(50)  # 50 Hz
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        
        if last_time is not None:
            # Calculate time delta
            dt = (current_time - last_time).to_sec()
            
            if dt > 0:
                # Calculate robot velocities using forward kinematics
                v_fl, v_rl, v_rr, v_fr = current_wheel_velocities
                v_x, v_y, omega_z = kin.forward_kinematics(v_fl, v_rl, v_rr, v_fr)
                
                # Update position using simple integration
                dx = v_x * dt
                dy = v_y * dt
                dyaw = omega_z * dt
                
                x += dx * np.cos(yaw) - dy * np.sin(yaw)
                y += dx * np.sin(yaw) + dy * np.cos(yaw)
                yaw += dyaw
                
                # Normalize yaw to [-pi, pi]
                yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        
        last_time = current_time
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        
        # Set position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set velocities (from forward kinematics)
        if last_time is not None:
            v_fl, v_rl, v_rr, v_fr = current_wheel_velocities
            v_x, v_y, omega_z = kin.forward_kinematics(v_fl, v_rl, v_rr, v_fr)
            
            odom.twist.twist.linear.x = v_x
            odom.twist.twist.linear.y = v_y
            odom.twist.twist.angular.z = omega_z
        
        # Set covariance (simple values)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.1  # yaw
        odom.twist.covariance[0] = 0.1  # v_x
        odom.twist.covariance[7] = 0.1  # v_y
        odom.twist.covariance[35] = 0.1 # omega_z
        
        # Publish odometry
        odom_pub.publish(odom)
        
        # Publish TF transform
        tf_broadcaster.sendTransform(
            (x, y, 0.0),
            quat,
            current_time,
            BASE_FRAME,
            ODOM_FRAME
        )
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

