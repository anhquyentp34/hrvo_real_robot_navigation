#!/usr/bin/env python3
"""
Goal sender với kịch bản waypoints (easy/medium/hard) và file YAML.
Dùng cho thí nghiệm so sánh Diffbot vs x_omni4wd; không sửa goal_sender_node.py (dùng ở dự án khác).

Tham số:
  ~scenario (str): easy | medium | hard → load config/waypoints_<scenario>.yaml
  ~waypoints_file (str): đường dẫn tuyệt đối tới file YAML (ưu tiên hơn scenario)
  ~num_cycles (int): số chu kỳ lặp waypoints
"""

import os
import rospy
import actionlib
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Waypoints mặc định (trung bình) khi không load được file
DEFAULT_WAYPOINTS = [
    {'name': 'GOAL_1', 'x': -2.0, 'y': 7.0, 'yaw': -1.57},
    {'name': 'GOAL_2', 'x': -4.5, 'y': -2.0, 'yaw': 0.0},
    {'name': 'GOAL_3', 'x': 10.0, 'y': -1.0, 'yaw': 3.14},
    {'name': 'GOAL_4', 'x': 3.0, 'y': 7.0, 'yaw': -1.57},
    {'name': 'GOAL_5', 'x': 3.0, 'y': 2.0, 'yaw': 1.57},
    {'name': 'GOAL_6', 'x': -10.0, 'y': -7.0, 'yaw': 1.57},
]


def load_waypoints_from_yaml(filepath):
    """Đọc waypoints từ file YAML. Trả về list dict với key name, x, y, yaw."""
    if not filepath or not os.path.isfile(filepath):
        return None
    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        wp_list = data.get('waypoints', data) if isinstance(data, dict) else data
        if not wp_list:
            return None
        return [
            {
                'name': str(w.get('name', 'GOAL_{}'.format(i + 1))),
                'x': float(w['x']),
                'y': float(w['y']),
                'yaw': float(w['yaw']),
            }
            for i, w in enumerate(wp_list)
        ]
    except Exception as e:
        rospy.logwarn("load_waypoints_from_yaml failed: %s", e)
        return None


def get_waypoints_config_path(scenario):
    """Trả về đường dẫn file config waypoints theo scenario (easy/medium/hard)."""
    try:
        import rospkg
        pkg_path = rospkg.RosPack().get_path('goal_sender')
        return os.path.join(pkg_path, 'config', 'waypoints_{}.yaml'.format(scenario.lower()))
    except Exception:
        return None


class GoalSenderScenario:
    def __init__(self):
        rospy.init_node('goal_sender_node_scenario')
        rospy.loginfo("Goal sender (scenario) node initialized")

        scenario = rospy.get_param('~scenario', 'medium')
        waypoints_file = rospy.get_param('~waypoints_file', '')
        self.num_cycles = int(rospy.get_param('~num_cycles', 1))

        self.waypoints = None
        if waypoints_file:
            self.waypoints = load_waypoints_from_yaml(waypoints_file)
            if self.waypoints:
                rospy.loginfo("Loaded %d waypoints from file: %s", len(self.waypoints), waypoints_file)
        if self.waypoints is None:
            config_path = get_waypoints_config_path(scenario)
            self.waypoints = load_waypoints_from_yaml(config_path) if config_path else None
            if self.waypoints:
                rospy.loginfo("Loaded %d waypoints for scenario '%s'", len(self.waypoints), scenario)
        if self.waypoints is None:
            self.waypoints = DEFAULT_WAYPOINTS
            rospy.loginfo("Using default (medium) waypoints, count=%d", len(self.waypoints))

        self.current_cycle = 0
        self.current_goal_index = 0
        self.goal_active = False
        self.last_goal_id = None
        self.active_goal_timeout = float(rospy.get_param('~active_goal_timeout', 0.0))
        self.active_goal_start_time = None  # thời điểm goal chuyển sang ACTIVE
        # Chỉ bỏ goal ABORTED khi proxemic đã nới tới ngưỡng max.
        self.abort_only_when_proxemic_max = bool(
            rospy.get_param('~abort_only_when_proxemic_max', True)
        )
        self.proxemic_max_weight = float(rospy.get_param('~proxemic_max_weight', 50.0))
        self.proxemic_max_tolerance = float(rospy.get_param('~proxemic_max_tolerance', 1e-3))
        self.proxemic_weight_param_global = rospy.get_param(
            '~proxemic_weight_param_global',
            '/move_base/global_costmap/proxemic/proxemic_density_weight',
        )
        self.proxemic_weight_param_local = rospy.get_param(
            '~proxemic_weight_param_local',
            '/move_base/local_costmap/proxemic/proxemic_density_weight',
        )
        self.abort_retry_delay_sec = float(rospy.get_param('~abort_retry_delay_sec', 0.5))

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.marker_pub = rospy.Publisher('/visualization_markers_start_goal', MarkerArray, queue_size=10)
        rospy.loginfo("Publisher for markers initialized")

        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        rospy.sleep(1.0)
        self.send_next_goal()

    def _read_proxemic_weights(self):
        values = []
        for key in (self.proxemic_weight_param_global, self.proxemic_weight_param_local):
            if rospy.has_param(key):
                try:
                    values.append(float(rospy.get_param(key)))
                except Exception:
                    pass
        return values

    def _proxemic_reached_max(self):
        values = self._read_proxemic_weights()
        if not values:
            # Nếu chưa đọc được param, không chặn luồng bình thường.
            return True
        threshold = self.proxemic_max_weight - self.proxemic_max_tolerance
        return min(values) >= threshold

    def send_next_goal(self):
        if self.current_goal_index < len(self.waypoints):
            wp = self.waypoints[self.current_goal_index]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = wp['x']
            goal_pose.pose.position.y = wp['y']
            goal_pose.pose.position.z = 0.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, wp['yaw'])
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            goal = MoveBaseGoal()
            goal.target_pose = goal_pose

            rospy.loginfo(
                "Sending goal %d/%d (Cycle %d/%d): %s at (%.2f, %.2f, %.2f)",
                self.current_goal_index + 1, len(self.waypoints),
                self.current_cycle + 1, self.num_cycles,
                wp['name'], wp['x'], wp['y'], wp['yaw']
            )
            self.move_base_client.send_goal(goal)
            # Không dùng gh nội bộ (dễ lỗi với một số bản actionlib/robot). Sẽ nhận goal_id từ /move_base/status (PENDING/ACTIVE).
            self.last_goal_id = None
            self.goal_active = True
            self.active_goal_start_time = None
            self.publish_current_goal_marker(wp)
        else:
            self.current_cycle += 1
            if self.current_cycle < self.num_cycles:
                rospy.loginfo("Completed cycle %d/%d, restarting waypoints.", self.current_cycle, self.num_cycles)
                self.current_goal_index = 0
                self.send_next_goal()
            else:
                rospy.loginfo("All cycles completed. Shutting down node.")
                rospy.signal_shutdown("All goals sent.")

    def status_callback(self, msg):
        """Cập nhật goal_id từ status PENDING/ACTIVE; khi goal kết thúc (2,3,4,5,8) thì gửi goal tiếp theo.
        Nếu active_goal_timeout > 0 và goal ACTIVE quá timeout giây mà không SUCCEEDED thì coi như tới đích (fallback)."""
        if not self.goal_active or not msg.status_list:
            return
        now = rospy.Time.now()
        for status in msg.status_list:
            if self.last_goal_id is None and status.status in [0, 1]:  # PENDING, ACTIVE -> đây là goal vừa gửi
                self.last_goal_id = status.goal_id.id
                if status.status == 1:  # ACTIVE
                    self.active_goal_start_time = now
            if self.last_goal_id and status.goal_id.id == self.last_goal_id:
                if status.status == 1:  # ACTIVE: cập nhật thời điểm, kiểm tra timeout
                    if self.active_goal_start_time is None:
                        self.active_goal_start_time = now
                    if self.active_goal_timeout > 0 and self.active_goal_start_time:
                        elapsed = (now - self.active_goal_start_time).to_sec()
                        if elapsed >= self.active_goal_timeout:
                            rospy.logwarn(
                                "Goal %d ACTIVE quá %.0fs không SUCCEEDED -> coi nhu toi dich (active_goal_timeout)",
                                self.current_goal_index + 1, elapsed
                            )
                            self.current_goal_index += 1
                            self.goal_active = False
                            self.last_goal_id = None
                            self.active_goal_start_time = None
                            rospy.sleep(0.5)
                            self.send_next_goal()
                            return
                elif status.status in [2, 3, 4, 5, 8]:  # PREEMPTED, SUCCEEDED, ABORTED, REJECTED, RECALLED
                    if status.status == 4 and self.abort_only_when_proxemic_max:
                        if not self._proxemic_reached_max():
                            values = self._read_proxemic_weights()
                            rospy.logwarn(
                                "Goal %d ABORTED nhưng proxemic chua max (global/local=%s < %.3f). "
                                "Gui lai cung goal thay vi chuyen goal tiep theo.",
                                self.current_goal_index + 1,
                                ",".join("{:.3f}".format(v) for v in values) if values else "n/a",
                                self.proxemic_max_weight,
                            )
                            self.goal_active = False
                            self.last_goal_id = None
                            self.active_goal_start_time = None
                            rospy.sleep(self.abort_retry_delay_sec)
                            self.send_next_goal()
                            return
                    rospy.loginfo("Goal %d finished with status: %d", self.current_goal_index + 1, status.status)
                    self.current_goal_index += 1
                    self.goal_active = False
                    self.last_goal_id = None
                    self.active_goal_start_time = None
                    rospy.sleep(0.5)
                    self.send_next_goal()
                break

    def publish_current_goal_marker(self, wp):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = wp['x']
        marker.pose.position.y = wp['y']
        marker.pose.position.z = 0.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "current_goal_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = wp['x']
        text_marker.pose.position.y = wp['y']
        text_marker.pose.position.z = 0.6
        text_marker.scale.z = 0.4
        text_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        text_marker.text = wp['name']
        text_marker.pose.orientation.w = 1.0
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        GoalSenderScenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
