#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import threading
import time

# Cấu hình cho từng actor
ACTOR_CONFIGS = [
    {
        'actor_id': 30,
        'path_topic': '/cmd_path',
        'waypoints': [(-10, 0.5), (10, 0.5)],
        'num_cycles': 10,
        'interval': 221.0
    },
    {
        'actor_id': 31,
        'path_topic': '/cmd_path2',
        'waypoints': [(-8, -0.5), (8, -0.5)],
        'num_cycles': 10,
        'interval': 161.0
    },
    {
        'actor_id': 32,
        'path_topic': '/cmd_path3',
        'waypoints': [(-10, 1.5), (-1, 1.5)],
        'num_cycles': 10,
        'interval': 91.0
    },
    {
        'actor_id': 33,
        'path_topic': '/cmd_path4',
        'waypoints': [(-9, -1.5), (-2, -1.5)],
        'num_cycles': 10,
        'interval': 70.0
    },
]

class ActorAutoPathPublisher:
    def __init__(self, actor_id, path_topic, waypoints, num_cycles, interval):
        self.actor_id = actor_id
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        self.waypoints = waypoints
        self.num_cycles = num_cycles
        self.interval = interval
        self.current_index = 0
        self.current_cycle = 0
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        rospy.loginfo(f"Actor {self.actor_id} auto path publisher initialized. {len(waypoints)} waypoints, {num_cycles} cycles, interval {interval}s.")
        self.thread.start()

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.current_cycle >= self.num_cycles:
                rospy.loginfo(f"Actor {self.actor_id} reached max cycles. Stopping.")
                break
            
            # Chờ interval trước khi gửi mục tiêu mới (trừ lần đầu tiên)
            if self.current_index > 0 or self.current_cycle > 0:
                rospy.loginfo(f"Actor {self.actor_id} waiting {self.interval}s before next goal...")
                time.sleep(self.interval)
            
            goal = self.waypoints[self.current_index]
            self.publish_path_to_goal(goal)
            rospy.loginfo(f"Actor {self.actor_id} sent goal {self.current_index+1}/{len(self.waypoints)} in cycle {self.current_cycle+1}/{self.num_cycles}: {goal}")
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.current_index = 0
                self.current_cycle += 1

    def publish_path_to_goal(self, goal):
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id="map")
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position = Point(goal[0], goal[1], 0.0)
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        path.poses.append(pose)
        self.path_pub.publish(path)
        rospy.loginfo(f"Actor {self.actor_id} published path to {goal}")

def main():
    rospy.init_node('multi_actor_path_publisher_node')
    actors = []
    for cfg in ACTOR_CONFIGS:
        actor = ActorAutoPathPublisher(
            actor_id=cfg['actor_id'],
            path_topic=cfg['path_topic'],
            waypoints=cfg['waypoints'],
            num_cycles=cfg['num_cycles'],
            interval=cfg['interval']
        )
        actors.append(actor)
    rospy.loginfo("Multi-actor auto path publisher started. Actors will send goals automatically.")
    rospy.spin()

if __name__ == '__main__':
    main() 