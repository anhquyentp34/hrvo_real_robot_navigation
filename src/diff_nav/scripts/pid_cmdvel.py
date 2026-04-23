#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PID:
    def __init__(self, kp, ki, kd, umin, umax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.umin = umin
        self.umax = umax
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def update(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3

        self.integral += error * dt

        derivative = 0.0
        if self.has_prev:
            derivative = (error - self.prev_error) / dt

        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        u = max(self.umin, min(self.umax, u))

        self.prev_error = error
        self.has_prev = True
        return u


class CmdVelPIDNode:
    def __init__(self):
        rospy.init_node("cmdvel_pid_node")

        self.v_max = rospy.get_param("~v_max", 0.8)
        self.w_max = rospy.get_param("~w_max", 1.5)
        self.rate = rospy.get_param("~rate", 30.0)

        kp_v = rospy.get_param("~kp_v", 1.0)
        ki_v = rospy.get_param("~ki_v", 0.0)
        kd_v = rospy.get_param("~kd_v", 0.05)

        kp_w = rospy.get_param("~kp_w", 1.5)
        ki_w = rospy.get_param("~ki_w", 0.0)
        kd_w = rospy.get_param("~kd_w", 0.05)

        self.pid_v = PID(kp_v, ki_v, kd_v, -self.v_max, self.v_max)
        self.pid_w = PID(kp_w, ki_w, kd_w, -self.w_max, self.w_max)

        self.v_ref = 0.0
        self.w_ref = 0.0
        self.v_meas = 0.0
        self.w_meas = 0.0

        self.have_ref = False
        self.have_odom = False

        self.last_time = rospy.Time.now()

        rospy.Subscriber("/cmd_vel_ref", Twist, self.ref_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.pub = rospy.Publisher("/cmd_vel_out", Twist, queue_size=1)

    def ref_cb(self, msg):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z
        self.have_ref = True

    def odom_cb(self, msg):
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z
        self.have_odom = True

    def run(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if dt <= 0.0:
                dt = 1.0 / self.rate

            out = Twist()

            if self.have_ref and self.have_odom:
                e_v = self.v_ref - self.v_meas
                e_w = self.w_ref - self.w_meas

                dv = self.pid_v.update(e_v, dt)
                dw = self.pid_w.update(e_w, dt)

                # kiểu 1: PID thuần tracking
                out.linear.x = max(-self.v_max, min(self.v_max, self.v_meas + dv))
                out.angular.z = max(-self.w_max, min(self.w_max, self.w_meas + dw))

                # nếu không muốn chạy lùi
                if out.linear.x < 0.0:
                    out.linear.x = 0.0

            self.pub.publish(out)
            r.sleep()


if __name__ == "__main__":
    try:
        CmdVelPIDNode().run()
    except rospy.ROSInterruptException:
        pass
