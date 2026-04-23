#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

class SimplePathPublisher:
    def __init__(self):
        rospy.init_node('simple_path_publisher', anonymous=True)
        
        # Tạo publishers cho 4 actor
        self.path_pubs = []
        for i in range(1, 5):
            pub = rospy.Publisher(f'/cmd_path_{i}', Path, queue_size=10)
            self.path_pubs.append(pub)
        
        # Định nghĩa vị trí xuất phát và mục tiêu cho từng actor
        self.start_positions = [
            (-5, 5),    # Doctor 1: góc tây bắc
            (5, 5),     # Doctor 2: góc đông bắc  
            (-5, -5),   # Doctor 3: góc tây nam
            (5, -5)     # Doctor 4: góc đông nam
        ]
        
        self.target_positions = [
            (-5, -5),    # Doctor 1 đi đến góc tây nam
            (-5, 5),     # Doctor 2 đi đến góc tây bắc
            (5, -5),     # Doctor 3 đi đến góc đông nam
            (5, 5)       # Doctor 4 đi đến góc đông bắc
        ]
        
        # Trạng thái của từng actor: 0 = đang đi đến mục tiêu, 1 = đang quay về
        self.actor_states = [0, 0, 0, 0]
        
        # Thời gian bắt đầu cho từng actor
        self.start_times = [None, None, None, None]
        
        # Thời gian cần thiết để đi từ start đến target (ước tính)
        self.travel_time = 15.0  # 15 giây
        
        self.rate = rospy.Rate(1)  # Publish mỗi giây
        
    def create_target_path(self, start_pos, target_pos):
        """Tạo path chỉ đến mục tiêu"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # Điểm mục tiêu
        target_pose = PoseStamped()
        target_pose.header = path.header
        target_pose.pose.position = Point(target_pos[0], target_pos[1], 0.0)
        
        # Tính hướng nhìn về mục tiêu
        dx = target_pos[0] - start_pos[0]
        dy = target_pos[1] - start_pos[1]
        yaw = math.atan2(dy, dx)
        target_pose.pose.orientation = Quaternion(0, 0, math.sin(yaw/2), math.cos(yaw/2))
        path.poses.append(target_pose)
        
        return path
    
    def create_return_path(self, start_pos, target_pos):
        """Tạo path quay về điểm xuất phát"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # Quay về điểm xuất phát
        return_pose = PoseStamped()
        return_pose.header = path.header
        return_pose.pose.position = Point(start_pos[0], start_pos[1], 0.0)
        
        # Tính hướng nhìn về điểm xuất phát
        dx = start_pos[0] - target_pos[0]
        dy = start_pos[1] - target_pos[1]
        yaw = math.atan2(dy, dx)
        return_pose.pose.orientation = Quaternion(0, 0, math.sin(yaw/2), math.cos(yaw/2))
        path.poses.append(return_pose)
        
        return path
    
    def run(self):
        rospy.loginfo("Simple Path Publisher started")
        
        # Đợi Gazebo và plugin khởi tạo xong
        rospy.loginfo("Waiting for Gazebo to initialize...")
        rospy.sleep(5.0)  # Đợi 5 giây
        
        # Bắt đầu với việc gửi tất cả actor đến mục tiêu
        for i in range(4):
            path = self.create_target_path(
                self.start_positions[i], 
                self.target_positions[i]
            )
            self.path_pubs[i].publish(path)
            self.start_times[i] = rospy.Time.now()
            rospy.loginfo(f"Doctor {i+1}: Starting to target {self.target_positions[i]}")
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            for i in range(4):
                if self.actor_states[i] == 0:  # Đang đi đến mục tiêu
                    # Kiểm tra xem đã đến lúc quay về chưa
                    if (current_time - self.start_times[i]).to_sec() >= self.travel_time:
                        # Gửi lệnh quay về
                        path = self.create_return_path(
                            self.start_positions[i], 
                            self.target_positions[i]
                        )
                        self.path_pubs[i].publish(path)
                        self.actor_states[i] = 1
                        rospy.loginfo(f"Doctor {i+1}: Returning to start {self.start_positions[i]}")
                
                elif self.actor_states[i] == 1:  # Đang quay về
                    # Kiểm tra xem đã về đến điểm xuất phát chưa
                    if (current_time - self.start_times[i]).to_sec() >= self.travel_time * 2:
                        # Bắt đầu chu kỳ mới
                        path = self.create_target_path(
                            self.start_positions[i], 
                            self.target_positions[i]
                        )
                        self.path_pubs[i].publish(path)
                        self.actor_states[i] = 0
                        self.start_times[i] = rospy.Time.now()
                        rospy.loginfo(f"Doctor {i+1}: Starting new cycle to target {self.target_positions[i]}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = SimplePathPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 