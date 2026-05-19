#!/usr/bin/env python3
import math
import threading

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from pedsim_msgs.msg import AgentStates
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

from hrvo_local_planner.msg import HRVOAgent, HRVOInput


class HRVOInputFusionNode:
    def __init__(self):
        self._pedsim_topic = rospy.get_param("~pedsim_topic", "/pedsim_simulator/simulated_agents")
        self._odom_topic = rospy.get_param("~odom_topic", "/odom")
        self._amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self._robot_pose_source = str(rospy.get_param("~robot_pose_source", "odom")).lower()
        self._global_path_topic = rospy.get_param("~global_path_topic", "/move_base/NavfnROS/plan")
        self._global_path_fallback_topic = rospy.get_param(
            "~global_path_fallback_topic", "/move_base/GlobalPlanner/plan"
        )
        if rospy.has_param("~lookahead_dist"):
            self._lookahead_dist = float(rospy.get_param("~lookahead_dist"))
        else:
            self._lookahead_dist = float(
                rospy.get_param("/move_base/HRVOLocalPlanner/lookahead_dist", 0.8)
            )
        self._max_agent_distance = float(rospy.get_param("~max_agent_distance", 8.0))
        self._output_topic = rospy.get_param("~output_topic", "/hrvo/input")
        self._target_marker_topic = rospy.get_param("~target_marker_topic", "/hrvo/target_marker")
        self._output_frame = rospy.get_param("~output_frame", "map")
        self._publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self._agent_radius = float(rospy.get_param("~agent_radius", 0.38))
        self._agent_type_whitelist = rospy.get_param("~agent_type_whitelist", [0, 1, 3])
        self._drop_robot_agent_type = bool(rospy.get_param("~drop_robot_agent_type", True))
        self._use_static_obstacles = bool(rospy.get_param("~use_static_obstacles", True))
        self._static_obstacle_map_topic = rospy.get_param(
            "~static_obstacle_map_topic", "/move_base/local_costmap/costmap"
        )
        self._static_cost_threshold = int(rospy.get_param("~static_cost_threshold", 90))
        self._static_sample_step = max(1, int(rospy.get_param("~static_sample_step", 2)))
        self._static_obstacle_radius = float(rospy.get_param("~static_obstacle_radius", 0.20))
        self._static_max_agents = max(0, int(rospy.get_param("~static_max_agents", 300)))

        self._lock = threading.Lock()
        self._last_agents = None
        self._last_odom = None
        self._last_amcl = None
        self._last_global_path = None
        self._last_global_path_stamp = rospy.Time(0)
        self._static_obstacles_xy = []
        self._static_obstacles_frame = self._output_frame
        self._static_obstacles_stamp = rospy.Time(0)

        self._pub = rospy.Publisher(self._output_topic, HRVOInput, queue_size=10)
        self._target_marker_pub = rospy.Publisher(self._target_marker_topic, Marker, queue_size=1)
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.Subscriber(self._pedsim_topic, AgentStates, self._on_agents, queue_size=1)
        rospy.Subscriber(self._odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(self._amcl_topic, PoseWithCovarianceStamped, self._on_amcl, queue_size=1)
        rospy.Subscriber(self._global_path_topic, Path, self._on_global_path, queue_size=1)
        if self._global_path_fallback_topic and self._global_path_fallback_topic != self._global_path_topic:
            rospy.Subscriber(
                self._global_path_fallback_topic, Path, self._on_global_path, queue_size=1
            )
        if self._use_static_obstacles:
            rospy.Subscriber(
                self._static_obstacle_map_topic, OccupancyGrid, self._on_static_obstacle_map, queue_size=1
            )

        period = 1.0 / max(self._publish_rate, 0.5)
        rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "hrvo_input_fusion_node: pedsim=%s odom=%s -> %s @ %.1f Hz",
            self._pedsim_topic,
            self._odom_topic,
            self._output_topic,
            self._publish_rate,
        )
        if self._use_static_obstacles:
            rospy.loginfo(
                "hrvo_input_fusion_node: static obstacles from %s (thr=%d, step=%d, max=%d)",
                self._static_obstacle_map_topic,
                self._static_cost_threshold,
                self._static_sample_step,
                self._static_max_agents,
            )

    def _transform_pose(self, pose, src_frame, stamp):
        if not src_frame or src_frame == self._output_frame:
            return pose
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = src_frame
        ps.pose = pose
        try:
            dst = self._tf_buffer.transform(ps, self._output_frame, rospy.Duration(0.05))
            return dst.pose
        except Exception:
            return None

    def _rotate_twist_xy(self, vx, vy, src_frame, stamp):
        if not src_frame or src_frame == self._output_frame:
            return vx, vy
        try:
            tr = self._tf_buffer.lookup_transform(
                self._output_frame, src_frame, stamp, rospy.Duration(0.05)
            )
            q = tr.transform.rotation
            # 2D yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            c = math.cos(yaw)
            s = math.sin(yaw)
            return (c * vx - s * vy, s * vx + c * vy)
        except Exception:
            return None

    def _on_agents(self, msg):
        with self._lock:
            self._last_agents = msg

    def _on_odom(self, msg):
        with self._lock:
            self._last_odom = msg

    def _on_amcl(self, msg):
        with self._lock:
            self._last_amcl = msg

    def _on_global_path(self, msg):
        if msg is None or not msg.poses:
            return
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        with self._lock:
            if stamp >= self._last_global_path_stamp:
                self._last_global_path = msg
                self._last_global_path_stamp = stamp

    def _on_static_obstacle_map(self, msg):
        if msg is None or msg.info.width <= 0 or msg.info.height <= 0:
            return
        width = int(msg.info.width)
        height = int(msg.info.height)
        if len(msg.data) < width * height:
            return

        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        sample = self._static_sample_step
        thr = self._static_cost_threshold

        pts = []
        for gy in range(0, height, sample):
            row_base = gy * width
            wy = oy + (gy + 0.5) * res
            for gx in range(0, width, sample):
                v = int(msg.data[row_base + gx])
                if v >= thr:
                    wx = ox + (gx + 0.5) * res
                    pts.append((wx, wy))
                    if self._static_max_agents > 0 and len(pts) >= self._static_max_agents:
                        break
            if self._static_max_agents > 0 and len(pts) >= self._static_max_agents:
                break

        with self._lock:
            self._static_obstacles_xy = pts
            self._static_obstacles_frame = (
                msg.header.frame_id if msg.header.frame_id else self._output_frame
            )
            self._static_obstacles_stamp = (
                msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            )

    def _transform_xy(self, x, y, src_frame, stamp):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = 0.0
        p.orientation.w = 1.0
        pose_out = self._transform_pose(p, src_frame, stamp)
        if pose_out is None:
            return None
        return (float(pose_out.position.x), float(pose_out.position.y))

    def _publish_target_marker(self, target):
        if self._target_marker_pub is None:
            return
        m = Marker()
        m.header.frame_id = target.header.frame_id if target.header.frame_id else self._output_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "hrvo_target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = target.pose
        m.scale.x = 0.30
        m.scale.y = 0.30
        m.scale.z = 0.30
        m.color.a = 0.95
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.lifetime = rospy.Duration(0.2)
        self._target_marker_pub.publish(m)

    def _accept_agent(self, a):
        if self._drop_robot_agent_type and int(a.type) == 2:
            return False
        if self._agent_type_whitelist:
            return int(a.type) in set(int(x) for x in self._agent_type_whitelist)
        return True

    @staticmethod
    def _dist2(a, b):
        dx = a.x - b.x
        dy = a.y - b.y
        return dx * dx + dy * dy

    def _pick_target_from_path(self, path_msg, robot_pose):
        if path_msg is None or not path_msg.poses:
            return None

        nearest_idx = 0
        best_d2 = float("inf")
        for i, ps in enumerate(path_msg.poses):
            d2 = self._dist2(ps.pose.position, robot_pose.position)
            if d2 < best_d2:
                best_d2 = d2
                nearest_idx = i

        for i in range(nearest_idx, len(path_msg.poses)):
            ps = path_msg.poses[i]
            dx = ps.pose.position.x - robot_pose.position.x
            dy = ps.pose.position.y - robot_pose.position.y
            if math.hypot(dx, dy) >= self._lookahead_dist:
                return ps

        return path_msg.poses[-1]

    def _is_agent_near_robot(self, pose_out, robot_pose_out):
        if self._max_agent_distance <= 0.0:
            return True
        dx = float(pose_out.position.x - robot_pose_out.position.x)
        dy = float(pose_out.position.y - robot_pose_out.position.y)
        return math.hypot(dx, dy) <= self._max_agent_distance

    def _is_xy_near_robot(self, x, y, robot_pose_out):
        if self._max_agent_distance <= 0.0:
            return True
        dx = float(x - robot_pose_out.position.x)
        dy = float(y - robot_pose_out.position.y)
        return math.hypot(dx, dy) <= self._max_agent_distance

    def _on_timer(self, _evt):
        with self._lock:
            agents_msg = self._last_agents
            odom_msg = self._last_odom
            amcl_msg = self._last_amcl
            global_path_msg = self._last_global_path
            static_obstacles_xy = list(self._static_obstacles_xy)
            static_obstacles_frame = self._static_obstacles_frame
            static_obstacles_stamp = self._static_obstacles_stamp

        if odom_msg is None:
            return

        out = HRVOInput()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self._output_frame

        robot_pose_out = None
        robot_twist_out = None

        if self._robot_pose_source == "amcl":
            if amcl_msg is None:
                return
            amcl_frame = amcl_msg.header.frame_id if amcl_msg.header.frame_id else "map"
            amcl_stamp = amcl_msg.header.stamp if amcl_msg.header.stamp != rospy.Time() else rospy.Time.now()
            robot_pose_out = self._transform_pose(amcl_msg.pose.pose, amcl_frame, amcl_stamp)
            if odom_msg is not None:
                robot_twist_out = odom_msg.twist.twist
        else:
            odom_frame = odom_msg.header.frame_id if odom_msg.header.frame_id else "odom"
            odom_stamp = odom_msg.header.stamp if odom_msg.header.stamp != rospy.Time() else rospy.Time.now()
            robot_pose_out = self._transform_pose(odom_msg.pose.pose, odom_frame, odom_stamp)
            robot_twist_out = odom_msg.twist.twist
            rotated = self._rotate_twist_xy(
                odom_msg.twist.twist.linear.x,
                odom_msg.twist.twist.linear.y,
                odom_frame,
                odom_stamp,
            )
            if rotated is not None:
                robot_twist_out.linear.x = rotated[0]
                robot_twist_out.linear.y = rotated[1]

        if robot_pose_out is None:
            return

        out.robot_pose = robot_pose_out
        out.robot_twist = robot_twist_out if robot_twist_out is not None else out.robot_twist

        target = self._pick_target_from_path(global_path_msg, out.robot_pose)
        if target is not None:
            if not target.header.frame_id:
                target.header.frame_id = self._output_frame
            if target.header.stamp == rospy.Time():
                target.header.stamp = out.header.stamp
            out.target = target
            self._publish_target_marker(target)

        if agents_msg is not None:
            agents_frame = agents_msg.header.frame_id if agents_msg.header.frame_id else self._output_frame
            agents_stamp = (
                agents_msg.header.stamp if agents_msg.header.stamp != rospy.Time() else rospy.Time.now()
            )
            for a in agents_msg.agent_states:
                if not self._accept_agent(a):
                    continue
                pose_out = self._transform_pose(a.pose, agents_frame, agents_stamp)
                if pose_out is None:
                    continue
                if not self._is_agent_near_robot(pose_out, out.robot_pose):
                    continue
                vel_out = self._rotate_twist_xy(
                    a.twist.linear.x, a.twist.linear.y, agents_frame, agents_stamp
                )
                if vel_out is None:
                    continue
                pa = HRVOAgent()
                pa.id = int(a.id)
                pa.x = float(pose_out.position.x)
                pa.y = float(pose_out.position.y)
                pa.vx = float(vel_out[0])
                pa.vy = float(vel_out[1])
                pa.radius = self._agent_radius
                pa.social_state = str(a.social_state)
                out.agents.append(pa)

        if self._use_static_obstacles and static_obstacles_xy:
            # id phải uint (ROS msg); dùng dải 100000+ cho vật cản tĩnh
            sid = 100000
            added = 0
            for (sx, sy) in static_obstacles_xy:
                sxy = self._transform_xy(sx, sy, static_obstacles_frame, static_obstacles_stamp)
                if sxy is None:
                    continue
                tx, ty = sxy
                if not self._is_xy_near_robot(tx, ty, out.robot_pose):
                    continue
                pa = HRVOAgent()
                pa.id = sid
                sid += 1
                pa.x = tx
                pa.y = ty
                pa.vx = 0.0
                pa.vy = 0.0
                pa.radius = self._static_obstacle_radius
                pa.social_state = "static_obstacle"
                out.agents.append(pa)
                added += 1
                if self._static_max_agents > 0 and added >= self._static_max_agents:
                    break
        elif self._use_static_obstacles:
            rospy.logwarn_throttle(
                5.0,
                "hrvo_input_fusion: chưa có ô costmap tĩnh (topic=%s, thr=%d)",
                self._static_obstacle_map_topic,
                self._static_cost_threshold,
            )

        self._pub.publish(out)


def main():
    rospy.init_node("hrvo_input_fusion_node")
    HRVOInputFusionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
