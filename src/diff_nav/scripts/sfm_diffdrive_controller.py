#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
import tf

class SFMDiffDriveController:
    def __init__(self):
        rospy.init_node("sfm_diffdrive_controller", anonymous=False)

        # ================================
        # ROBOT KINEMATICS
        # ================================
        self.WHEEL_SEP = 0.390
        self.L = self.WHEEL_SEP / 2.0
        self.WHEEL_RADIUS = 0.155 / 2.0

        self.V_WHEEL_MAX = 0.7

        # ================================
        # CONTROL LIMITS
        # ================================
        self.v_max = 0.6
        self.w_max = 1.0

        self.a_lin = 1.0
        self.a_ang = 0.7

        self.k_theta = 0.8
        self.THETA_DEADZONE = 0.05
        self.MAX_SPEED_REDUCTION = 0.5
        self.ANGLE_THRESHOLD_FOR_STOP_AND_TURN = math.radians(60.0)

        # ================================
        # STATE
        # ================================
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.yaw = 0.0
        self.last_time = None

        # ================================
        # SOCIAL MODE
        # 0 = polite | 1 = normal | 2 = assertive
        # ================================
        self.social_mode = 1  # NORMAL default

        # ================================
        # PUB / SUB
        # ================================
        self.cmd_pub = rospy.Publisher("/cmd_vel_sfm", Twist, queue_size=1)

        rospy.Subscriber("/sfm/velocity", Vector3, self.sfm_vel_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/social_mode", Int32, self.cb_social_mode)

        # episode reset (đồng bộ RL)
        rospy.Subscriber("/episode_done", Bool, self.cb_episode_done)

        rospy.loginfo("[SFM DiffDrive] Ready and synced with RL")
        rospy.spin()

    # ================================
    # CALLBACKS
    # ================================
    def cb_social_mode(self, msg):
        # clamp để an toàn
        self.social_mode = max(0, min(2, msg.data))

    def cb_episode_done(self, msg):
        if msg.data:
            # reset internal dynamics để tránh giật
            self.v_prev = 0.0
            self.w_prev = 0.0
            self.last_time = None

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.yaw = yaw

    # ================================
    # MAIN CONTROL
    # ================================
    def sfm_vel_cb(self, msg):
        if self.last_time is None:
            self.last_time = rospy.Time.now()
            return

        vx, vy = msg.x, msg.y
        v_sfm = math.hypot(vx, vy)

        # ===== SOCIAL MODULATION =====
        if self.social_mode == 0:      # polite
            v_sfm *= 0.5
        elif self.social_mode == 2:    # assertive
            v_sfm *= 1.2

        theta_des = math.atan2(vy, vx)
        e_theta = self.angle_diff(theta_des, self.yaw)

        # stop-and-turn logic
        if abs(e_theta) > self.ANGLE_THRESHOLD_FOR_STOP_AND_TURN:
            v_cmd_ideal = 0.0
        else:
            v_cmd_ideal = v_sfm * math.cos(e_theta)

        # angular control
        if abs(e_theta) < self.THETA_DEADZONE:
            w_cmd_ideal = 0.0
        else:
            w_cmd_ideal = self.k_theta * e_theta

        # diff-drive constraint
        v_r = v_cmd_ideal + w_cmd_ideal * self.L
        v_l = v_cmd_ideal - w_cmd_ideal * self.L

        v_r = max(-self.V_WHEEL_MAX, min(self.V_WHEEL_MAX, v_r))
        v_l = max(-self.V_WHEEL_MAX, min(self.V_WHEEL_MAX, v_l))

        v_cmd = (v_r + v_l) / 2.0
        w_cmd = (v_r - v_l) / self.WHEEL_SEP

        # speed reduction when turning
        if self.w_max > 0:
            turn_ratio = abs(w_cmd) / self.w_max
            v_cmd *= max(1.0 - turn_ratio * self.MAX_SPEED_REDUCTION,
                         1.0 - self.MAX_SPEED_REDUCTION)

        # social speed cap
        v_max_social = self.v_max
        if self.social_mode == 0:
            v_max_social = 0.4
        elif self.social_mode == 2:
            v_max_social = 0.8

        v_cmd = max(-v_max_social, min(v_max_social, v_cmd))
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))

        # time
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0:
            dt = 0.02

        # acceleration ramp
        dv = max(-self.a_lin * dt, min(self.a_lin * dt, v_cmd - self.v_prev))
        dw = max(-self.a_ang * dt, min(self.a_ang * dt, w_cmd - self.w_prev))

        self.v_prev += dv
        self.w_prev += dw

        # publish
        cmd = Twist()
        cmd.linear.x = self.v_prev
        cmd.angular.z = self.w_prev
        self.cmd_pub.publish(cmd)

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


if __name__ == "__main__":
    try:
        SFMDiffDriveController()
    except rospy.ROSInterruptException:
        pass

