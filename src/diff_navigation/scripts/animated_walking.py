#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from animated_marker_msgs.msg import AnimatedMarkerArray

def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

def norm(x, y): 
    return math.hypot(x, y)

def dot(ax, ay, bx, by):
    return ax*bx + ay*by

def cross2(ax, ay, bx, by):
    # 2D cross product z-component
    return ax*by - ay*bx

class CommitRightPredictive:
    def __init__(self):
        # --- tunables ---
        self.R = rospy.get_param("~R", 6.0)               # max range consider
        self.path_w = rospy.get_param("~path_w", 1.0)     # "duong" rong bao nhieu (m)
        self.ttc_h = rospy.get_param("~ttc_h", 4.0)       # time horizon (s)
        self.ttc_min = rospy.get_param("~ttc_min", 0.4)   # under this -> very dangerous
        self.k_w = rospy.get_param("~k_w", 1.6)           # turn gain
        self.k_v = rospy.get_param("~k_v", 0.7)           # slow-down gain
        self.dw_max = rospy.get_param("~dw_max", 1.2)     # max angular accel (rad/s^2)
        self.v_min = rospy.get_param("~v_min", 0.05)
        self.v_max = rospy.get_param("~v_max", 0.6)

        # smoothing / commit
        self.bias_tau = rospy.get_param("~bias_tau", 0.25)        # low-pass tau for bias
        self.commit_time = rospy.get_param("~commit_time", 0.8)   # keep avoiding for (s)
        self.commit_th_on = rospy.get_param("~commit_th_on", 0.35) # trigger threshold
        self.commit_th_off = rospy.get_param("~commit_th_off", 0.20)# release threshold

        # state
        self.px = self.py = 0.0
        self.vx = self.vy = 0.0   # robot vel from odom
        self.goal = None

        # tracking humans: id -> (x,y,vx,vy,t)
        self.trk = {}

        self.w_prev = 0.0
        self.t_prev = None
        self.bias_lp = 0.0

        self.committing = False
        self.commit_until = 0.0

        rospy.Subscriber("/odom", Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        rospy.Subscriber("/animated_human_tracks", AnimatedMarkerArray, self.cb_people, queue_size=1)
        rospy.Subscriber("/cmd_vel_sfm", Twist, self.cb_cmd, queue_size=1)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def cb_odom(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def cb_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def cb_people(self, msg):
        now = rospy.Time.now().to_sec()
        # note: comment khong dau theo yeu cau
        for m in msg.markers:
            hid = m.id
            x = m.pose.position.x
            y = m.pose.position.y

            if hid in self.trk:
                x0, y0, vx0, vy0, t0 = self.trk[hid]
                dt = max(1e-3, now - t0)
                vx = (x - x0) / dt
                vy = (y - y0) / dt
                # smooth vel a bit
                a = 0.4
                vx = (1-a)*vx0 + a*vx
                vy = (1-a)*vy0 + a*vy
                self.trk[hid] = (x, y, vx, vy, now)
            else:
                self.trk[hid] = (x, y, 0.0, 0.0, now)

        # optional: remove stale tracks
        stale = []
        for hid, (_, _, _, _, t0) in self.trk.items():
            if now - t0 > 1.0:
                stale.append(hid)
        for hid in stale:
            self.trk.pop(hid, None)

    def compute_risk(self, dirx, diry):
        """
        Return (risk, need_commit, w_bias_raw, v_scale)
        risk: 0..1
        w_bias_raw: positive means "push to right" (we will subtract from cmd.angular.z)
        v_scale: 0..1 multiply on linear speed
        """
        if not self.trk:
            return 0.0, False, 0.0, 1.0

        best_score = 0.0
        best_ttc = None
        best_side = 0.0

        # robot vel along heading dir (approx)
        vr = dot(self.vx, self.vy, dirx, diry)

        for hid, (hx, hy, hvx, hvy, _) in self.trk.items():
            rx = hx - self.px
            ry = hy - self.py
            d = norm(rx, ry)
            if d > self.R or d < 1e-3:
                continue

            # ahead check (along dir)
            along = dot(rx, ry, dirx, diry)
            if along < 0.0:
                continue

            # lateral offset to our path direction
            lat = abs(cross2(dirx, diry, rx, ry))
            if lat > self.path_w:
                continue

            # relative speed along path direction
            hv_along = dot(hvx, hvy, dirx, diry)
            rel_along = vr - hv_along

            # if we are not closing in, TTC not meaningful
            if rel_along <= 0.05:
                # still some risk if very close
                close_score = clamp((self.path_w - lat)/self.path_w, 0.0, 1.0) * clamp((self.R - d)/self.R, 0.0, 1.0)
                score = 0.3 * close_score
                if score > best_score:
                    best_score = score
                    best_ttc = None
                    best_side = cross2(dirx, diry, rx, ry)
                continue

            # TTC based on along distance / closing speed
            ttc = along / rel_along
            if ttc < 0.0 or ttc > self.ttc_h:
                continue

            # score uses TTC + distance + lateral
            ttc_score = clamp((self.ttc_h - ttc)/self.ttc_h, 0.0, 1.0)
            d_score = clamp((self.R - d)/self.R, 0.0, 1.0)
            lat_score = clamp((self.path_w - lat)/self.path_w, 0.0, 1.0)

            score = 0.55*ttc_score + 0.25*d_score + 0.20*lat_score

            if score > best_score:
                best_score = score
                best_ttc = ttc
                best_side = cross2(dirx, diry, rx, ry)  # + means target on left of dir, - means on right

        if best_score <= 1e-6:
            return 0.0, False, 0.0, 1.0

        # commit logic (hysteresis)
        need_commit = best_score >= self.commit_th_on

        # IMPORTANT: only push right strongly if obstacle is on left/center.
        # If obstacle already on right (best_side < 0), reduce pushing right to avoid over-steer.
        side_gain = 1.0
        if best_side < -0.05:
            side_gain = 0.35

        w_bias_raw = self.k_w * best_score * side_gain

        # slow down if TTC is small
        v_scale = 1.0
        if best_ttc is not None:
            danger = clamp((self.ttc_h - best_ttc)/self.ttc_h, 0.0, 1.0)
            # extra danger if under ttc_min
            if best_ttc < self.ttc_min:
                danger = 1.0
            v_scale = clamp(1.0 - self.k_v * danger, 0.15, 1.0)

        return best_score, need_commit, w_bias_raw, v_scale

    def cb_cmd(self, cmd):
        now = rospy.Time.now().to_sec()
        if self.t_prev is None:
            self.t_prev = now
        dt = max(1e-3, now - self.t_prev)
        self.t_prev = now

        out = Twist()

        # default: passthrough
        v_cmd = clamp(cmd.linear.x, -self.v_max, self.v_max)
        w_cmd = cmd.angular.z

        w_bias = 0.0
        v_scale = 1.0

        if self.goal is not None:
            gx, gy = self.goal
            dx, dy = gx - self.px, gy - self.py
            g = norm(dx, dy)
            if g > 1e-3:
                dirx, diry = dx/g, dy/g

                risk, need_commit, w_bias_raw, v_scale = self.compute_risk(dirx, diry)

                # commit state machine
                if self.committing:
                    if now > self.commit_until and risk < self.commit_th_off:
                        self.committing = False
                else:
                    if need_commit:
                        self.committing = True
                        self.commit_until = now + self.commit_time

                if self.committing:
                    # low-pass the bias for smoothness
                    alpha = clamp(dt / max(1e-3, self.bias_tau), 0.0, 1.0)
                    self.bias_lp = (1-alpha)*self.bias_lp + alpha*w_bias_raw
                    w_bias = self.bias_lp
                else:
                    # decay bias smoothly to 0
                    alpha = clamp(dt / max(1e-3, self.bias_tau), 0.0, 1.0)
                    self.bias_lp = (1-alpha)*self.bias_lp
                    w_bias = self.bias_lp

        # apply: subtract bias => steer right
        w_target = w_cmd - w_bias

        # angular accel limit (smooth)
        dw = clamp(w_target - self.w_prev, -self.dw_max*dt, self.dw_max*dt)
        self.w_prev += dw
        out.angular.z = self.w_prev

        # speed scaling for safety
        v_out = clamp(v_cmd * v_scale, self.v_min, self.v_max)
        out.linear.x = v_out

        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("commit_right_predictive")
    CommitRightPredictive()
    rospy.spin()

