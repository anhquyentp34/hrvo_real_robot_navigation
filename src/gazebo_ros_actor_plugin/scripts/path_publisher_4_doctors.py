#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

def publish_paths():
    # Initialize the node
    rospy.init_node('path_publisher_4_doctors_node')
    
    # Create publishers for each doctor
    pub1 = rospy.Publisher('/cmd_path_1', Path, queue_size=1, latch=True)
    pub2 = rospy.Publisher('/cmd_path_2', Path, queue_size=1, latch=True)
    pub3 = rospy.Publisher('/cmd_path_3', Path, queue_size=1, latch=True)
    pub4 = rospy.Publisher('/cmd_path_4', Path, queue_size=1, latch=True)
    
    rate = rospy.Rate(1)  # Publish at 1 Hz
    
    while not rospy.is_shutdown():
        # Doctor 1: Đi theo hình vuông từ góc tây bắc (-5, 5)
        path1 = create_square_path_from_start(-5, 5, 2)
        path1.header.stamp = rospy.Time.now()
        path1.header.frame_id = "map"
        pub1.publish(path1)
        
        # Doctor 2: Đi theo hình tròn từ góc đông bắc (5, 5)
        path2 = create_circle_path_from_start(5, 5, 2)
        path2.header.stamp = rospy.Time.now()
        path2.header.frame_id = "map"
        pub2.publish(path2)
        
        # Doctor 3: Đi theo đường thẳng từ góc tây nam (-5, -5)
        path3 = create_straight_path_from_start(-5, -5, 4)
        path3.header.stamp = rospy.Time.now()
        path3.header.frame_id = "map"
        pub3.publish(path3)
        
        # Doctor 4: Đi theo hình tam giác từ góc đông nam (5, -5)
        path4 = create_triangle_path_from_start(5, -5, 2)
        path4.header.stamp = rospy.Time.now()
        path4.header.frame_id = "map"
        pub4.publish(path4)
        
        rate.sleep()

def create_square_path_from_start(start_x, start_y, size):
    """Tạo đường đi hình vuông bắt đầu từ vị trí khởi tạo"""
    path = Path()
    poses = []
    
    # Bắt đầu từ vị trí khởi tạo, sau đó đi theo hình vuông
    # Vị trí khởi tạo: (start_x, start_y)
    # Các điểm của hình vuông:
    points = [
        (start_x, start_y),                    # Điểm khởi đầu
        (start_x + size, start_y),             # Đi về phía đông
        (start_x + size, start_y - size),      # Đi về phía nam
        (start_x, start_y - size),             # Đi về phía tây
        (start_x, start_y),                    # Quay lại điểm đầu
    ]
    
    for i, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Tính hướng nhìn về điểm tiếp theo
        if i < len(points) - 1:
            next_x, next_y = points[i + 1]
            angle = math.atan2(next_y - y, next_x - x)
        else:
            angle = math.atan2(points[0][1] - y, points[0][0] - x)
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

def create_circle_path_from_start(start_x, start_y, radius):
    """Tạo đường đi hình tròn bắt đầu từ vị trí khởi tạo"""
    path = Path()
    poses = []
    
    # Bắt đầu từ vị trí khởi tạo
    center_x = start_x - radius  # Tâm hình tròn ở bên trái điểm khởi đầu
    center_y = start_y
    
    # Thêm điểm khởi đầu
    start_pose = PoseStamped()
    start_pose.header.stamp = rospy.Time.now()
    start_pose.header.frame_id = "map"
    start_pose.pose.position = Point(x=start_x, y=start_y, z=0)
    start_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
    poses.append(start_pose)
    
    # Tạo 12 điểm trên đường tròn (mượt hơn)
    for i in range(12):
        angle = i * 2 * math.pi / 12
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Hướng tiếp tuyến với đường tròn
        tangent_angle = angle + math.pi/2
        quat = Quaternion(*quaternion_from_euler(0, 0, tangent_angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    # Quay lại vị trí khởi đầu
    end_pose = PoseStamped()
    end_pose.header.stamp = rospy.Time.now()
    end_pose.header.frame_id = "map"
    end_pose.pose.position = Point(x=start_x, y=start_y, z=0)
    end_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
    poses.append(end_pose)
    
    path.poses = poses
    return path

def create_straight_path_from_start(start_x, start_y, length):
    """Tạo đường đi thẳng bắt đầu từ vị trí khởi tạo"""
    path = Path()
    poses = []
    
    # Điểm đầu và điểm cuối
    points = [
        (start_x, start_y),           # Vị trí khởi đầu
        (start_x + length, start_y),  # Đi về phía đông
        (start_x, start_y),           # Quay lại vị trí khởi đầu
    ]
    
    for i, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Hướng về phía trước hoặc sau
        if i == 0:
            angle = 0  # Nhìn về phía đông
        elif i == 1:
            angle = math.pi  # Nhìn về phía tây
        else:
            angle = 0  # Nhìn về phía đông
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

def create_triangle_path_from_start(start_x, start_y, size):
    """Tạo đường đi hình tam giác bắt đầu từ vị trí khởi tạo"""
    path = Path()
    poses = []
    
    # Bắt đầu từ vị trí khởi tạo, sau đó đi theo hình tam giác đều
    # Vị trí khởi tạo: (start_x, start_y)
    # Các đỉnh của tam giác đều:
    points = [
        (start_x, start_y),                    # Điểm khởi đầu
        (start_x + size, start_y),             # Đỉnh phải
        (start_x + size/2, start_y - size * math.sqrt(3)/2),  # Đỉnh dưới
        (start_x, start_y),                    # Quay lại điểm đầu
    ]
    
    for i, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Tính hướng nhìn về điểm tiếp theo
        if i < len(points) - 1:
            next_x, next_y = points[i + 1]
            angle = math.atan2(next_y - y, next_x - x)
        else:
            angle = math.atan2(points[0][1] - y, points[0][0] - x)
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

# Giữ lại các hàm cũ để tương thích (nếu cần)
def create_square_path(center_x, center_y, size):
    """Tạo đường đi hình vuông"""
    path = Path()
    poses = []
    
    # 4 góc của hình vuông
    corners = [
        (center_x - size, center_y + size),  # Tây bắc
        (center_x + size, center_y + size),  # Đông bắc
        (center_x + size, center_y - size),  # Đông nam
        (center_x - size, center_y - size),  # Tây nam
        (center_x - size, center_y + size),  # Quay lại điểm đầu
    ]
    
    for i, (x, y) in enumerate(corners):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Tính hướng nhìn về điểm tiếp theo
        if i < len(corners) - 1:
            next_x, next_y = corners[i + 1]
            angle = math.atan2(next_y - y, next_x - x)
        else:
            angle = math.atan2(corners[0][1] - y, corners[0][0] - x)
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

def create_circle_path(center_x, center_y, radius):
    """Tạo đường đi hình tròn"""
    path = Path()
    poses = []
    
    # Tạo 8 điểm trên đường tròn
    for i in range(8):
        angle = i * 2 * math.pi / 8
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Hướng tiếp tuyến với đường tròn
        tangent_angle = angle + math.pi/2
        quat = Quaternion(*quaternion_from_euler(0, 0, tangent_angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    # Thêm điểm đầu để tạo vòng tròn hoàn chỉnh
    poses.append(poses[0])
    path.poses = poses
    return path

def create_straight_path(start_x, start_y, length):
    """Tạo đường đi thẳng"""
    path = Path()
    poses = []
    
    # Điểm đầu và điểm cuối
    points = [
        (start_x, start_y),
        (start_x + length, start_y),
        (start_x, start_y),
    ]
    
    for i, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Hướng về phía trước hoặc sau
        if i == 0:
            angle = 0  # Nhìn về phía đông
        else:
            angle = math.pi  # Nhìn về phía tây
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

def create_triangle_path(center_x, center_y, size):
    """Tạo đường đi hình tam giác"""
    path = Path()
    poses = []
    
    # 3 đỉnh của tam giác
    vertices = [
        (center_x, center_y + size),  # Đỉnh trên
        (center_x + size, center_y - size),  # Đỉnh phải dưới
        (center_x - size, center_y - size),  # Đỉnh trái dưới
        (center_x, center_y + size),  # Quay lại đỉnh đầu
    ]
    
    for i, (x, y) in enumerate(vertices):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=0)
        
        # Tính hướng nhìn về điểm tiếp theo
        if i < len(vertices) - 1:
            next_x, next_y = vertices[i + 1]
            angle = math.atan2(next_y - y, next_x - x)
        else:
            angle = math.atan2(vertices[0][1] - y, vertices[0][0] - x)
        
        quat = Quaternion(*quaternion_from_euler(0, 0, angle))
        pose.pose.orientation = quat
        poses.append(pose)
    
    path.poses = poses
    return path

if __name__ == '__main__':
    try:
        publish_paths()
    except rospy.ROSInterruptException:
        pass 