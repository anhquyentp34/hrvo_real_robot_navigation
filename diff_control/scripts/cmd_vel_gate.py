#!/usr/bin/env python3
"""
Giới hạn và timeout cho geometry_msgs/Twist trước khi gửi tới Gazebo diff_drive.
Robot diff chỉ dùng linear.x và angular.z; linear.y luôn ép về 0.

Cấu trúc tương tự x_omni4wd_control/scripts/cmd_vel_to_omni_wheels.py (timeout + clamp),
nhưng không tách IK bánh (plugin libgazebo_ros_diff_drive xử lý).
"""
import rospy
from geometry_msgs.msg import Twist


class CmdVelGate:
    def __init__(self):
        rospy.init_node("cmd_vel_gate", anonymous=True)

        self.max_linear_x = rospy.get_param("~max_linear_x", 0.8)
        self.max_linear_y = rospy.get_param("~max_linear_y", 0.0)
        self.max_angular_z = rospy.get_param("~max_angular_z", 1.2)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        in_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        out_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self._last_cmd = rospy.Time.now()
        self._pub = rospy.Publisher(out_topic, Twist, queue_size=1)
        rospy.Subscriber(in_topic, Twist, self._cb, queue_size=5)

        rospy.loginfo(
            "cmd_vel_gate: %s -> %s | max vx=%.3f wz=%.3f timeout=%.2fs",
            in_topic,
            out_topic,
            self.max_linear_x,
            self.max_angular_z,
            self.cmd_timeout,
        )

    def _clamp(self, v, lim):
        if lim <= 0.0:
            return 0.0
        return max(-lim, min(lim, v))

    def _cb(self, msg):
        self._last_cmd = rospy.Time.now()
        out = Twist()
        out.linear.x = self._clamp(msg.linear.x, self.max_linear_x)
        out.linear.y = self._clamp(msg.linear.y, self.max_linear_y)
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self._clamp(msg.angular.z, self.max_angular_z)
        self._pub.publish(out)

    def run(self):
        rate = rospy.Rate(50)
        zero = Twist()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self._last_cmd).to_sec() > self.cmd_timeout:
                self._pub.publish(zero)
                rospy.logwarn_throttle(
                    2.0,
                    "cmd_vel_gate: timeout %.2fs, publishing zero",
                    self.cmd_timeout,
                )
            rate.sleep()


if __name__ == "__main__":
    try:
        n = CmdVelGate()
        n.run()
    except rospy.ROSInterruptException:
        pass
