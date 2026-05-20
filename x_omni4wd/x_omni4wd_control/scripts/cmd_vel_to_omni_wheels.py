#!/usr/bin/env python3
"""
ROS node to convert cmd_vel (geometry_msgs/Twist) to wheel angular velocity commands
for 4-wheel omni/X-drive robot using ros_control.

This node subscribes to /cmd_vel and publishes to individual wheel controller command topics.
Uses standard Jacobian formula for X-drive configuration (ROS REP-103: +y Left, +w CCW).

Reference: MSD_H20_OMNI_Bridge.ino - Jacobian formula for X-drive omni robot
  w_FL = (vx - vy - k*w) / r
  w_RL = (vx + vy - k*w) / r
  w_RR = (-vx + vy - k*w) / r
  w_FR = (-vx - vy - k*w) / r
Where:
  - vx: vận tốc theo trục X (Forward), m/s
  - vy: vận tốc theo trục Y (Left), m/s
  - w: vận tốc góc (CCW dương), rad/s
  - r: bán kính bánh xe, m
  - k: hệ số quay = L_HALF + W_HALF, m
  - L_HALF: nửa chiều dài robot (tâm -> trục bánh trước/sau theo x)
  - W_HALF: nửa chiều rộng robot (tâm -> trục bánh trái/phải theo y)

Mapping: LF→FL, RF→FR, LB→RL, RB→RR

Author: quyenanh pt <anhquyentp34@gmail.com>
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToOmniWheels:
    def __init__(self):
        rospy.init_node('cmd_vel_to_omni_wheels', anonymous=True)
        
        # Get parameters with defaults
        # L_HALF: nửa chiều dài robot (tâm -> trục bánh trước/sau theo x)
        self.L_HALF = rospy.get_param('~L_HALF', 0.176776695)  # m
        # W_HALF: nửa chiều rộng robot (tâm -> trục bánh trái/phải theo y)
        self.W_HALF = rospy.get_param('~W_HALF', 0.176776695)  # m
        # Wheel radius
        self.r = rospy.get_param('~wheel_radius', 0.0635)  # m
        
        # Calculate k = L_HALF + W_HALF (hệ số quay)
        self.k = self.L_HALF + self.W_HALF
        
        # Clamp ω bánh (rad/s). Launch sim/navigation thường đặt ~22 để khớp TEB; bringup ros_control vẫn có thể truyền 40.
        self.max_wheel_speed = rospy.get_param('~max_wheel_speed', 40.0)  # rad/s
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)  # seconds
        
        # Last command time
        self.last_cmd_time = rospy.Time.now()
        
        # Publishers for wheel velocity commands
        # Using namespace /x_omni4wd as specified
        self.pub_fl = rospy.Publisher('/x_omni4wd/front_left_wheel_velocity_controller/command',
                                      Float64, queue_size=1)
        self.pub_rl = rospy.Publisher('/x_omni4wd/rear_left_wheel_velocity_controller/command',
                                      Float64, queue_size=1)
        self.pub_rr = rospy.Publisher('/x_omni4wd/rear_right_wheel_velocity_controller/command',
                                      Float64, queue_size=1)
        self.pub_fr = rospy.Publisher('/x_omni4wd/front_right_wheel_velocity_controller/command',
                                      Float64, queue_size=1)

        # Subscriber for cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Log parameters
        rospy.loginfo("cmd_vel_to_omni_wheels node initialized (Jacobian X-drive formula):")
        rospy.loginfo("  L_HALF = %.5f m (nửa chiều dài robot)", self.L_HALF)
        rospy.loginfo("  W_HALF = %.5f m (nửa chiều rộng robot)", self.W_HALF)
        rospy.loginfo("  k = L_HALF + W_HALF = %.5f m (hệ số quay)", self.k)
        rospy.loginfo("  wheel_radius = %.5f m", self.r)
        rospy.loginfo("  max_wheel_speed = %.2f rad/s", self.max_wheel_speed)
        rospy.loginfo("  cmd_timeout = %.2f s", self.cmd_timeout)
        rospy.loginfo("  Jacobian: w_FL=(vx-vy-k*w)/r, w_RL=(vx+vy-k*w)/r, w_RR=(-vx+vy-k*w)/r, w_FR=(-vx-vy-k*w)/r")
        
    def cmd_vel_callback(self, msg):
        """
        Inverse kinematics X-drive (ROS REP-103: +x forward, +y left, +w CCW).
        
        Công thức (khớp firmware MSD_H20_OMNI_Bridge.ino):
          w_FL = (vx - vy - k*w) / r
          w_RL = (vx + vy - k*w) / r
          w_RR = (-vx + vy - k*w) / r
          w_FR = (-vx - vy - k*w) / r
        
        Đảm bảo (vx=0, vy, w=0) → tịnh tiến ngang (sang trái nếu vy>0, sang phải nếu vy<0), không quay.
        """
        self.last_cmd_time = rospy.Time.now()
        
        # Giữ theo ROS REP-103 cho cmd_vel nhận vào:
        #   linear.x > 0  => robot theo +X (tùy cấu hình robot thực tế, có thể cần hiệu chỉnh ở IK bên dưới)
        #   linear.y > 0  => robot theo +Y
        #   angular.z > 0 => quay theo +w (CCW trong REP-103)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Inverse kinematics X-drive (ROS REP-103: +x forward, +y left, +w CCW).
        # Dạng chuẩn đối xứng 4 bánh:
     
        w_fl = (-vx + vy + self.k * wz) / self.r        # Front-left
        w_rl = (-vx - vy + self.k * wz) / self.r        # Rear-left
        w_rr = (vx - vy + self.k * wz) / self.r         # Rear-right
        w_fr = (vx + vy + self.k * wz) / self.r         # Front-right
        
        # Clamp to max wheel speed
        w_fl = max(-self.max_wheel_speed, min(self.max_wheel_speed, w_fl))
        w_rl = max(-self.max_wheel_speed, min(self.max_wheel_speed, w_rl))
        w_rr = max(-self.max_wheel_speed, min(self.max_wheel_speed, w_rr))
        w_fr = max(-self.max_wheel_speed, min(self.max_wheel_speed, w_fr))
        
        # Publish wheel velocity commands
        self.pub_fl.publish(Float64(w_fl))
        self.pub_rl.publish(Float64(w_rl))
        self.pub_rr.publish(Float64(w_rr))
        self.pub_fr.publish(Float64(w_fr))
        
    def run(self):
        """Main loop with timeout handling"""
        rate = rospy.Rate(50)  # 50 Hz
        
        while not rospy.is_shutdown():
            # Check for timeout
            elapsed = (rospy.Time.now() - self.last_cmd_time).to_sec()
            if elapsed > self.cmd_timeout:
                # Send zero velocities
                self.pub_fl.publish(Float64(0.0))
                self.pub_rl.publish(Float64(0.0))
                self.pub_rr.publish(Float64(0.0))
                self.pub_fr.publish(Float64(0.0))
                
                if elapsed > self.cmd_timeout + 0.1:  # Log warning only occasionally
                    rospy.logwarn_throttle(1.0, "cmd_vel timeout (%.2f s), sending zero velocities", elapsed)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelToOmniWheels()
        node.run()
    except rospy.ROSInterruptException:
        pass

