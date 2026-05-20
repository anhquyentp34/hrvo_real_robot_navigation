#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RViz: AnimatedMarkerArray từ /gazebo/model_states, giống ICR2026 (x_omni4wd_simulation/actor_visualization_node.py)
nhưng mặc định khớp Pedsim: tên model Gazebo là id số (1, 2, 3, …), frame world.

Tham số giữ nguyên tên như bản ICR2026 để dùng chung rosparam / RViz.
"""
from __future__ import print_function

import math
import re

import rospy
import tf.transformations as tft
from animated_marker_msgs.msg import AnimatedMarker, AnimatedMarkerArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from pedsim_msgs.msg import AgentStates
from visualization_msgs.msg import Marker, MarkerArray


def _stable_actor_id(name):
    digits = "".join(c for c in name if c.isdigit())
    return int(digits) if digits else (hash(name) & 0x7FFFFFFF)


class ActorVisualizationNode(object):
    def __init__(self):
        self._input_source = rospy.get_param("~input_source", "pedsim").strip().lower()
        self._model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self._pedsim_topic = rospy.get_param("~pedsim_topic", "/pedsim_simulator/simulated_agents")
        self._output_topic = rospy.get_param("~output_topic", "/actor_visualization")
        self._frame_id = rospy.get_param("~frame_id", "world")
        self._publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self._stationary_speed_threshold = float(rospy.get_param("~stationary_speed_threshold", 0.05))
        self._stationary_animation_speed = float(rospy.get_param("~stationary_animation_speed", 0.0))
        default_moving_mesh = rospy.get_param(
            "~human_mesh_resource",
            "package://animated_marker_tutorial/meshes/animated_walking_man.mesh",
        )
        self._moving_human_mesh_resource = rospy.get_param("~moving_human_mesh_resource", default_moving_mesh)
        self._standing_human_mesh_resource = rospy.get_param(
            "~standing_human_mesh_resource",
            default_moving_mesh,
        )
        self._moving_human_mesh_scale = float(rospy.get_param("~human_mesh_scale", 0.38))
        self._moving_human_mesh_z_offset = float(rospy.get_param("~human_mesh_z_offset", 0.0))
        self._standing_human_mesh_scale = float(rospy.get_param("~standing_human_mesh_scale", 0.9))
        self._standing_human_mesh_z_offset = float(rospy.get_param("~standing_human_mesh_z_offset", 0.0))
        self._show_name_text = bool(rospy.get_param("~show_name_text", True))
        self._use_actor_unique_colors = bool(rospy.get_param("~use_actor_unique_colors", True))
        self._mesh_use_embedded_materials = bool(rospy.get_param("~mesh_use_embedded_materials", False))
        actor_regex = rospy.get_param("~actor_name_regex", r"^[0-9]+$")
        self._actor_re = re.compile(actor_regex)
        static_actor_regex = rospy.get_param("~static_actor_name_regex", r"^$")
        self._static_actor_re = re.compile(static_actor_regex)
        self._color_palette = [
            (0.90, 0.25, 0.25),
            (0.25, 0.55, 0.95),
            (0.20, 0.75, 0.35),
            (0.95, 0.65, 0.20),
            (0.70, 0.40, 0.95),
            (0.15, 0.75, 0.75),
            (0.95, 0.40, 0.70),
            (0.70, 0.70, 0.20),
            (0.95, 0.55, 0.35),
            (0.45, 0.65, 0.95),
        ]

        self._last_pos = {}
        self._latest_models = None
        self._latest_agents = None
        self._pub = rospy.Publisher(self._output_topic, AnimatedMarkerArray, queue_size=1)
        self._text_pub = None
        if self._show_name_text:
            self._text_pub = rospy.Publisher(self._output_topic + "_text", MarkerArray, queue_size=1)

        if self._input_source == "gazebo":
            rospy.Subscriber(self._model_states_topic, ModelStates, self._cb_models, queue_size=1)
        else:
            rospy.Subscriber(self._pedsim_topic, AgentStates, self._cb_agents, queue_size=1)
        period = 1.0 / max(self._publish_rate, 0.5)
        rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "pedsim_actor_visualization_node: source=%s (%s) -> %s frame=%s regex=%s mesh=%s",
            self._input_source,
            self._pedsim_topic if self._input_source != "gazebo" else self._model_states_topic,
            self._output_topic,
            self._frame_id,
            actor_regex,
            self._moving_human_mesh_resource,
        )

    def _cb_models(self, msg):
        self._latest_models = msg

    def _cb_agents(self, msg):
        self._latest_agents = msg

    def _on_timer(self, _evt):
        if self._input_source == "gazebo":
            entries = self._iter_entries_from_gazebo()
        else:
            entries = self._iter_entries_from_pedsim()
        if entries is None:
            return
        now = rospy.Time.now()
        arr = AnimatedMarkerArray()
        text_arr = MarkerArray() if self._show_name_text else None

        for name, pose in entries:
            pos = pose.position
            ori = pose.orientation
            q = [ori.x, ori.y, ori.z, ori.w]
            roll, pitch, yaw = tft.euler_from_quaternion(q)
            theta_deg = yaw * 180.0 / math.pi

            speed = 0.0
            if name in self._last_pos:
                lx, ly, lz, lt = self._last_pos[name]
                dt = (now - lt).to_sec()
                if dt > 1e-6:
                    speed = math.sqrt((pos.x - lx) ** 2 + (pos.y - ly) ** 2) / dt
            self._last_pos[name] = (pos.x, pos.y, pos.z, now)

            if speed < self._stationary_speed_threshold:
                anim_speed = self._stationary_animation_speed
            else:
                anim_speed = 0.7 * min(speed, 3.0)
            is_static_actor = bool(self._static_actor_re.match(name))

            mid = _stable_actor_id(name)
            m = AnimatedMarker()
            m.header.frame_id = self._frame_id
            m.header.stamp = now
            m.ns = "actors"
            m.id = mid
            m.type = AnimatedMarker.MESH_RESOURCE
            m.action = AnimatedMarker.ADD
            m.pose.position.x = pos.x
            m.pose.position.y = pos.y
            if is_static_actor:
                m.pose.position.z = pos.z + self._standing_human_mesh_z_offset
                q_fix = tft.quaternion_from_euler(0.0, 0.0, yaw + 1.5708)
                mesh_scale = self._standing_human_mesh_scale
            else:
                m.pose.position.z = pos.z + self._moving_human_mesh_z_offset
                q_fix = tft.quaternion_from_euler(math.pi / 2.0, 0.0, (theta_deg + 90.0) / 180.0 * math.pi)
                mesh_scale = self._moving_human_mesh_scale
            m.pose.orientation = Quaternion(q_fix[0], q_fix[1], q_fix[2], q_fix[3])
            m.scale.x = m.scale.y = m.scale.z = mesh_scale
            if self._use_actor_unique_colors:
                cr, cg, cb = self._color_palette[mid % len(self._color_palette)]
            else:
                cr, cg, cb = (0.85, 0.85, 0.88)
            m.color.r = cr
            m.color.g = cg
            m.color.b = cb
            m.color.a = 1.0
            m.lifetime = rospy.Duration(0)
            m.frame_locked = True
            m.mesh_use_embedded_materials = self._mesh_use_embedded_materials
            if is_static_actor:
                m.mesh_resource = self._standing_human_mesh_resource
                m.animation_speed = 0.0
            else:
                m.mesh_resource = self._moving_human_mesh_resource
                m.animation_speed = anim_speed
            arr.markers.append(m)

            if text_arr is not None:
                tm = Marker()
                tm.header.frame_id = self._frame_id
                tm.header.stamp = now
                tm.ns = "actor_names"
                tm.id = 10000 + mid
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = pos.x
                tm.pose.position.y = pos.y
                text_z_offset = self._standing_human_mesh_z_offset if is_static_actor else self._moving_human_mesh_z_offset
                tm.pose.position.z = pos.z + text_z_offset + 1.15
                tm.pose.orientation.w = 1.0
                tm.scale.z = 0.35
                tm.color.r = 1.0
                tm.color.g = 1.0
                tm.color.b = 1.0
                tm.color.a = 1.0
                tm.text = name
                tm.lifetime = rospy.Duration(0)
                text_arr.markers.append(tm)

        self._pub.publish(arr)
        if text_arr is not None and self._text_pub is not None:
            self._text_pub.publish(text_arr)

    def _iter_entries_from_gazebo(self):
        if self._latest_models is None:
            return None
        out = []
        for i, name in enumerate(self._latest_models.name):
            if "_collision" in name:
                continue
            if not self._actor_re.match(name):
                continue
            out.append((name, self._latest_models.pose[i]))
        return out

    def _iter_entries_from_pedsim(self):
        if self._latest_agents is None:
            return None
        out = []
        for agent in self._latest_agents.agent_states:
            if int(agent.type) == 2:
                continue
            name = str(agent.id)
            if not self._actor_re.match(name):
                continue
            out.append((name, agent.pose))
        return out


if __name__ == "__main__":
    rospy.init_node("pedsim_actor_visualization_node", anonymous=False)
    ActorVisualizationNode()
    rospy.spin()
