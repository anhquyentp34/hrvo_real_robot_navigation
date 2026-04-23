#!/usr/bin/env python3
import rospy, threading, math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from gazebo_msgs.msg import ModelStates  # cần gazebo_ros
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

FRAME_ID = "world"
REACH_EPS = 0.25  # khoảng cách coi là đã tới đích (m)

class ActorAutoPathPublisher:
    def __init__(self, actor_name, path_topic, waypoints):
        self.actor_name = actor_name
        self.pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        self.waypoints = waypoints
        self.idx = 0
        self.cur_goal = None
        self.actor_xy = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._states_cb)
        # gửi goal đầu tiên
        rospy.Timer(rospy.Duration(0.5), self._control_loop)

    def _states_cb(self, msg):
        # lấy vị trí actor theo tên (vd: "actor30")
        if self.actor_name in msg.name:
            i = msg.name.index(self.actor_name)
            p = msg.pose[i].position
            self.actor_xy = (p.x, p.y)

    def _publish_goal(self, xy):
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id=FRAME_ID)
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position = Point(xy[0], xy[1], 0.0)
        ps.pose.orientation = Quaternion(*quaternion_from_euler(0,0,0))
        path.poses = [ps]  # một điểm đích là đủ
        self.pub.publish(path)
        self.cur_goal = xy
        rospy.loginfo(f"[{self.actor_name}] new goal -> {xy}")

    def _control_loop(self, _):
        if self.cur_goal is None:
            self._publish_goal(self.waypoints[self.idx])
            return
        if self.actor_xy is None:
            return
        dist = math.hypot(self.cur_goal[0]-self.actor_xy[0], self.cur_goal[1]-self.actor_xy[1])
        if dist <= REACH_EPS:
            # chuyển sang waypoint tiếp theo
            self.idx = (self.idx + 1) % len(self.waypoints)
            self._publish_goal(self.waypoints[self.idx])

def main():
    rospy.init_node("multi_actor_path_publisher_node")

    actors = [
        # (tên actor trong world, topic path, danh sách waypoint)
        ("actor1", "/cmd_path1",  [(10, 0.7), (1, 0.7)]),
        ("actor2", "/cmd_path2", [(10, -0.7), (1,  -0.7)]),
        #("actor1", "/cmd_path1",  [(7, -1.0), (-6, -1.0)]),
        #("actor2", "/cmd_path2", [(0, 6.0), (1,  -9.0)]),
        #("actor3", "/cmd_path3", [(4, -5.0), (4,  -9.0)]),
    ]
    pubs = [ActorAutoPathPublisher(name, topic, wps) for name, topic, wps in actors]
    rospy.spin()

if __name__ == "__main__":
    main()

