#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# ---------------- utils ----------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)

def dot(ax, ay, bx, by):
    return ax*bx + ay*by

def normalize(x, y, eps=1e-9):
    n = math.hypot(x, y)
    if n < eps:
        return 0.0, 0.0, 0.0
    return x/n, y/n, n

# ---------------- node ----------------
class SocialDoorSupervisorSFM:
    """
    - input  : /cmd_vel_nav (move_base + SFM local planner)
    - output : /cmd_vel
    - human  : /gazebo/model_states (actor pose+vel)
    - robot  : /odom

    Goal: tránh xa người hơn (social), không đi sát người, ưu tiên né sang phải khi yield ở cửa.
    """

    def __init__(self):
        rospy.init_node("social_door_supervisor_sfm")

        # Topics
        self.cmd_nav_topic = rospy.get_param("~cmd_nav_topic", "/cmd_vel_nav")
        self.cmd_out_topic = rospy.get_param("~cmd_out_topic", "/cmd_vel")
        self.odom_topic    = rospy.get_param("~odom_topic", "/odom")
        self.states_topic  = rospy.get_param("~model_states_topic", "/gazebo/model_states")

        # Human model
        self.human_model = rospy.get_param("~human_model", "actor1")

        # Door
        self.door_x = rospy.get_param("~door_x", 5.6)
        self.door_y = rospy.get_param("~door_y", -1.0)
        self.door_yaw = rospy.get_param("~door_yaw", 0.0)
        self.delta_d = rospy.get_param("~delta_d", 1.8)  # sub-goal lùi trước cửa (x_sg=x_d-δ)
        self.r_in  = rospy.get_param("~door_radius_in", 0.5)
        self.r_out = rospy.get_param("~door_radius_out", 0.8)
        self.trigger_radius = rospy.get_param("~trigger_radius", 3.5)

        # Unsafe region Δh (door frame)
        self.unsafe_half_len = rospy.get_param("~unsafe_half_len", 2.6)
        self.unsafe_half_w   = rospy.get_param("~unsafe_half_w", 1.2)

        # Time cue
        self.eta_eps   = rospy.get_param("~eta_eps", 0.08)
        self.eta_hyst  = rospy.get_param("~eta_hyst", 1.0)
        self.eta_h_max = rospy.get_param("~eta_h_max", 8.0)

        # --- SOCIAL COMFORT (tăng để tránh xa) ---
        self.d_comfort = rospy.get_param("~d_comfort", 1.6)   # giữ khoảng cách thoải mái
        self.d_safe    = rospy.get_param("~d_safe",    2.2)   # bắt đầu tránh mạnh
        self.d_stop    = rospy.get_param("~d_stop",    1.0)   # dừng khẩn nếu quá gần
        self.d_min_pred = rospy.get_param("~d_min_pred", 1.2) # khoảng cách dự đoán tối thiểu (CPA) để không lướt sát

        # TTC / prediction
        self.ttc_th    = rospy.get_param("~ttc_th", 2.2)      # giây
        self.ttc_hard  = rospy.get_param("~ttc_hard", 1.0)    # giây
        self.pred_horizon = rospy.get_param("~pred_horizon", 3.0)

        # Avoid gains
        self.rep_gain_w = rospy.get_param("~rep_gain_w", 2.4)     # bẻ lái tránh
        self.rep_gain_v = rospy.get_param("~rep_gain_v", 0.85)    # giảm tốc khi gần
        self.right_bias = rospy.get_param("~right_bias", 0.55)    # ép né phải khi yield
        self.follow_right_always = rospy.get_param("~follow_right_always", True)

        # Sub-goal / controller (u=[v,w] trực tiếp)
        self.k_lin = rospy.get_param("~k_lin", 0.9)
        self.k_ang = rospy.get_param("~k_ang", 2.4)
        self.v_max = rospy.get_param("~v_max", 0.28)
        self.w_max = rospy.get_param("~w_max", 1.4)
        self.slow_r = rospy.get_param("~slow_radius", 1.0)
        self.reach_dist = rospy.get_param("~reach_dist", 0.30)

        # Resume ramp
        self.resume_ramp_s = rospy.get_param("~resume_ramp_s", 0.6)

        self.debug = rospy.get_param("~debug", False)

        # Door frame axis
        self.ax = math.cos(self.door_yaw)
        self.ay = math.sin(self.door_yaw)
        self.lx = -self.ay
        self.ly =  self.ax

        # IO state
        self._lock = threading.Lock()
        self._cmd_nav = Twist()
        self._robot_pose = None     # (x,y,yaw)
        self._robot_vxy  = (0.0, 0.0)
        self._human_xy = None
        self._human_v  = None

        # hysteresis
        self.human_in = False

        # state machine
        self.state = "NORMAL"       # NORMAL | GO_SG | STOP | WAIT | RESUME
        self._resume_t0 = None
        self._override_last = Twist()

        # subs/pubs
        rospy.Subscriber(self.cmd_nav_topic, Twist, self._cb_cmd_nav, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber(self.states_topic, ModelStates, self._cb_states, queue_size=1)
        self.pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=1)

    # ---------------- callbacks ----------------
    def _cb_cmd_nav(self, msg):
        with self._lock:
            self._cmd_nav = msg

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        with self._lock:
            self._robot_pose = (p.x, p.y, yaw)
            self._robot_vxy = (vx, vy)

    def _cb_states(self, msg: ModelStates):
        try:
            i = msg.name.index(self.human_model)
            hp = msg.pose[i].position
            hv = msg.twist[i].linear
            self._human_xy = (hp.x, hp.y)
            self._human_v  = (hv.x, hv.y)
        except ValueError:
            self._human_xy = None
            self._human_v = None

    # ---------------- geometry ----------------
    def _to_door_frame(self, x, y):
        tx = x - self.door_x
        ty = y - self.door_y
        s = dot(tx, ty, self.ax, self.ay)
        l = dot(tx, ty, self.lx, self.ly)
        return s, l

    def _in_unsafe(self, rx, ry):
        s, l = self._to_door_frame(rx, ry)
        return (abs(s) <= self.unsafe_half_len) and (abs(l) <= self.unsafe_half_w)

    def _human_near_door(self, hx, hy):
        return dist(hx, hy, self.door_x, self.door_y) <= self.trigger_radius

    def _update_human_in_door_hyst(self, hx, hy):
        d = dist(hx, hy, self.door_x, self.door_y)
        if self.human_in:
            if d >= self.r_out:
                self.human_in = False
        else:
            if d <= self.r_in:
                self.human_in = True
        return self.human_in, d

    # ---------------- paper sub-goal (Algorithm 2) ----------------
    def _sub_goal_paper(self):
        sgx = self.door_x - self.delta_d * self.ax
        sgy = self.door_y - self.delta_d * self.ay
        return sgx, sgy

    # ---------------- ETA + prediction ----------------
    def _eta(self, px, py, vx, vy):
        d = dist(px, py, self.door_x, self.door_y)
        sp = math.hypot(vx, vy)
        return d / max(sp, self.eta_eps), d, sp

    def _robot_eta_from_cmd(self, rx, ry, cmd_nav: Twist):
        d = dist(rx, ry, self.door_x, self.door_y)
        sp = abs(cmd_nav.linear.x)
        return d / max(sp, self.eta_eps), d, sp

    def _cpa(self, rx, ry, rvx, rvy, hx, hy, hvx, hvy):
        # closest point of approach under constant velocities
        dx = hx - rx
        dy = hy - ry
        dvx = hvx - rvx
        dvy = hvy - rvy
        dv2 = dvx*dvx + dvy*dvy
        if dv2 < 1e-6:
            return float("inf"), math.hypot(dx, dy)
        t = - (dx*dvx + dy*dvy) / dv2
        if t < 0.0:
            return float("inf"), math.hypot(dx, dy)
        if t > self.pred_horizon:
            t = self.pred_horizon
        cx = dx + dvx*t
        cy = dy + dvy*t
        dca = math.hypot(cx, cy)
        return t, dca

    # ---------------- controller u=[v,w] ----------------
    def _cmd_to_point(self, rx, ry, ryaw, tx, ty):
        dx = tx - rx
        dy = ty - ry
        d = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        err = angle_wrap(ang - ryaw)

        w = clamp(self.k_ang * err, -self.w_max, self.w_max)

        v = self.k_lin * d
        if d < self.slow_r:
            v *= d / max(1e-6, self.slow_r)
        if abs(err) > 0.9:
            v *= 0.2
        v = clamp(v, 0.0, self.v_max)

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        return out, d

    # ---------------- SOCIAL AVOID: giữ khoảng cách xa hơn ----------------
    def _social_avoid(self, u: Twist, rx, ry, ryaw, rvx, rvy, hx, hy, hvx, hvy, yield_mode: bool):
        d = dist(rx, ry, hx, hy)
        if d <= self.d_stop:
            return Twist()

        # CPA/TTC
        t_cpa, d_cpa = self._cpa(rx, ry, rvx, rvy, hx, hy, hvx, hvy)

        # 1) nếu dự đoán sẽ lướt sát -> ép tránh mạnh + giảm tốc mạnh
        pred_risk = (t_cpa < self.ttc_th) and (d_cpa < self.d_min_pred)

        # 2) risk theo khoảng cách hiện tại
        # scale in [0,1] when d in [d_safe -> d_comfort]
        close_now = clamp((self.d_safe - d) / max(self.d_safe - self.d_comfort, 1e-6), 0.0, 1.0)

        # 3) risk theo TTC
        ttc_risk = 0.0
        if t_cpa < self.ttc_th:
            ttc_risk = clamp((self.ttc_th - t_cpa) / self.ttc_th, 0.0, 1.0)

        risk = max(close_now, ttc_risk)
        if pred_risk:
            risk = max(risk, 0.85)

        # hướng human so với heading robot
        bearing = math.atan2(hy - ry, hx - rx)
        err = angle_wrap(bearing - ryaw)   # >0 human bên trái, <0 human bên phải

        # steer away from human
        steer_sign = -math.copysign(1.0, err)  # human left -> steer right

        # nếu luôn muốn đi bên phải người (social norm) thì ép steer về phải khi đối đầu
        # (khi human nằm gần hướng trước mặt robot)
        if self.follow_right_always:
            front = abs(err) < (math.radians(70))
            if front:
                steer_sign = -1.0  # ép rẽ phải

        w_add = self.rep_gain_w * risk * steer_sign

        # yield mode: bias rẽ phải mạnh hơn
        if yield_mode:
            w_add += -abs(self.right_bias)

        uu = Twist()
        uu.angular.z = clamp(u.angular.z + w_add, -self.w_max, self.w_max)

        # giảm tốc khi risk cao (giữ khoảng cách “xã hội”)
        v = u.linear.x
        if d < self.d_safe:
            # slowdown factor in (0.1..1.0)
            slow = clamp((d - self.d_stop) / max(self.d_safe - self.d_stop, 1e-6), 0.1, 1.0)
            v *= (1.0 - self.rep_gain_v * (1.0 - slow))
        if pred_risk:
            v *= 0.35

        # nếu TTC rất nhỏ, giảm tiếp
        if t_cpa < self.ttc_hard:
            v *= 0.25

        uu.linear.x = clamp(v, 0.0, self.v_max)
        return uu

    # ---------------- main ----------------
    def spin(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            with self._lock:
                cmd_nav = self._cmd_nav
                rpose = self._robot_pose
                rvx, rvy = self._robot_vxy
                hxy = self._human_xy
                hv  = self._human_v

            if rpose is None:
                self.pub.publish(cmd_nav)
                rate.sleep()
                continue

            rx, ry, ryaw = rpose
            rvx_w = rvx * math.cos(ryaw) - rvy * math.sin(ryaw)
            rvy_w = rvx * math.sin(ryaw) + rvy * math.cos(ryaw)
            if hxy is None or hv is None:
                self.pub.publish(cmd_nav)
                rate.sleep()
                continue

            hx, hy = hxy
            hvx, hvy = hv

            in_door, d_hdoor = self._update_human_in_door_hyst(hx, hy)
            near_door = self._human_near_door(hx, hy)

            # time cue: human_first
            eta_h, _, _ = self._eta(hx, hy, hvx, hvy)
            eta_r, _, _ = self._robot_eta_from_cmd(rx, ry, cmd_nav)
            human_first = (eta_h + self.eta_hyst) < eta_r and (eta_h <= self.eta_h_max)

            # paper Algorithm 3 trigger
            need_yield = near_door and human_first

            if self.debug:
                rospy.loginfo_throttle(
                    0.5,
                    f"[social] d={dist(rx,ry,hx,hy):.2f} cpa={self._cpa(rx,ry,rvx_w,rvy_w,hx,hy,hvx,hvy)[1]:.2f} "
                    f"nearDoor={near_door} inDoor={in_door} eta_h={eta_h:.2f} eta_r={eta_r:.2f} "
                    f"human_first={human_first} unsafe={self._in_unsafe(rx,ry)} state={self.state}"
                )

            # -------- transitions (Algorithm 3 style) --------
            if self.state == "NORMAL":
                if need_yield:
                    if self._in_unsafe(rx, ry):
                        self.state = "STOP"
                    else:
                        self.state = "GO_SG"

            elif self.state == "GO_SG":
                if need_yield and self._in_unsafe(rx, ry):
                    self.state = "STOP"
                if (not in_door) and (not need_yield):
                    self.state = "RESUME"
                    self._resume_t0 = rospy.Time.now()

            elif self.state == "STOP":
                if (not in_door) and (not need_yield):
                    self.state = "RESUME"
                    self._resume_t0 = rospy.Time.now()
                elif need_yield and (not self._in_unsafe(rx, ry)):
                    self.state = "GO_SG"

            elif self.state == "WAIT":
                if (not in_door) and (not need_yield):
                    self.state = "RESUME"
                    self._resume_t0 = rospy.Time.now()

            elif self.state == "RESUME":
                if self._resume_t0 is not None:
                    dt = (rospy.Time.now() - self._resume_t0).to_sec()
                    if dt >= self.resume_ramp_s:
                        self.state = "NORMAL"
                        self._resume_t0 = None

            # human đến cửa sau -> không yield
            if near_door and (not human_first) and self.state in ("GO_SG", "STOP", "WAIT"):
                self.state = "RESUME"
                self._resume_t0 = rospy.Time.now()

            # -------- output u=[v,w] --------
            if self.state == "NORMAL":
                # vẫn áp social avoid để không đi sát người
                out = self._social_avoid(cmd_nav, rx, ry, ryaw, rvx_w, rvy_w, hx, hy, hvx, hvy, yield_mode=False)

            elif self.state == "GO_SG":
                sgx, sgy = self._sub_goal_paper()
                u, dsg = self._cmd_to_point(rx, ry, ryaw, sgx, sgy)
                u = self._social_avoid(u, rx, ry, ryaw, rvx_w, rvy_w, hx, hy, hvx, hvy, yield_mode=True)
                self._override_last = u
                if dsg <= self.reach_dist:
                    self.state = "WAIT"
                    out = Twist()
                    self._override_last = out
                else:
                    out = u

            elif self.state == "STOP":
                out = Twist()
                self._override_last = out

            elif self.state == "WAIT":
                out = Twist()
                self._override_last = out

            else:  # RESUME
                if self._resume_t0 is None or self.resume_ramp_s <= 1e-3:
                    out = cmd_nav
                else:
                    dt = (rospy.Time.now() - self._resume_t0).to_sec()
                    a = clamp(dt / self.resume_ramp_s, 0.0, 1.0)
                    out = Twist()
                    out.linear.x  = (1.0 - a) * self._override_last.linear.x  + a * cmd_nav.linear.x
                    out.angular.z = (1.0 - a) * self._override_last.angular.z + a * cmd_nav.angular.z

            self.pub.publish(out)
            rate.sleep()

if __name__ == "__main__":
    SocialDoorSupervisorSFM().spin()

