#!/usr/bin/python3

# ... (Phần imports giữ nguyên)
import rospy
import ros_numpy
import numpy as np
import tf2_ros
import math
from tf.transformations import quaternion_matrix, quaternion_from_euler
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
from scipy.spatial import KDTree
import sys
from scipy.optimize import linear_sum_assignment

from animated_marker_msgs.msg import AnimatedMarker, AnimatedMarkerArray
import tf2_geometry_msgs as tf_geo
from geometry_msgs.msg import PointStamped 

# ===============================================
#          ĐỊNH NGHĨA HẰNG SỐ TOÀN CỤC (Khắc phục NameError)
# ===============================================
GLOBAL_OFFSET_X = 0.0
GLOBAL_OFFSET_Y = 0.0
GLOBAL_ANGLE = 0.0
FIXED_FRAME_ID = "base_footprint" 
TARGET_FRAME_ID = "map"          
DOWNSAMPLE_CELL_SIZE = 0.15
ROOM_X_MIN = -6.0
ROOM_X_MAX = 6.0
ROOM_Y_MIN = -2.0
ROOM_Y_MAX = 2.0
GROUND_Z_MAX = 0.3 # VỊ TRÍ CHÍNH XÁC
SENSOR_HEIGHT = 1.3
WALL_POINT_SPACING = 0.5
HUMAN_MIN_POINTS = 4
HUMAN_MAX_POINTS = 7000
HUMAN_MIN_HEIGHT = 0.5
HUMAN_MAX_HEIGHT = 2.5
HUMAN_MAX_DIAMETER = 1.5
MIN_DETECTION_DISTANCE = 0.4
CLOSE_DETECTION_RADIUS = 2.5
CLOSE_MIN_POINTS = 3
CLOSE_MIN_HEIGHT = 0.1
CLUSTER_EPS = 0.45
CLUSTER_MIN_SAMPLES = 3
TRACK_MAX_DIST = 15.0
TRACK_MAX_MISSES = 10
WALL_PROXIMITY_THRESHOLD = 0.35 
HUMAN_MESH_RESOURCE = "package://animated_marker_tutorial/meshes/animated_walking_man.mesh"
HUMAN_MESH_SCALE = 0.4 
MARKER_LIFETIME = 0.5 # Tăng để ổn định

# ===============================================
#                 CLASS DEFINITIONS
# ===============================================

# --- KALMAN FILTER CLASS (Giữ nguyên) ---
class SimpleKalmanFilter:
    def __init__(self, initial_pos):
        self.x = np.array([initial_pos[0], initial_pos[1], 0, 0])
        self.F = np.eye(4)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.P = np.eye(4) * 1.0
        self.R = np.eye(2) * 0.1
        self.Q = np.eye(4) * 0.1

    def predict(self, dt):
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, measurement):
        z = np.array(measurement)
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        return self.x

# --- Track Class (ĐÃ SỬA: Thêm vị trí Map Frame) ---
class Track:
    """A class to represent a single tracked object (human or wall)."""
    def __init__(self, track_id, initial_pos, is_human=True):
        self.id = track_id
        self.is_human = is_human
        self.missed_frames = 0
        self.pos_map_x = 0.0 # Vị trí map frame cuối cùng đã biết
        self.pos_map_y = 0.0
        
        if self.is_human:
            self.kf = SimpleKalmanFilter(initial_pos[0:2])
            self.pos = initial_pos[0:2] # Vị trí trong base_footprint (KF frame)
            self.vel = np.array([0.0, 0.0])
            self.color = (np.random.rand(), np.random.rand(), np.random.rand())
        else:
            self.pos = initial_pos[0:2]
            self.vel = np.array([0.0, 0.0])
            self.color = (0.5, 0.5, 0.5)

    def update(self, new_detection, dt):
        if not self.is_human: return
        self.kf.predict(dt)
        measurement = new_detection[0:2]
        updated_state = self.kf.update(measurement)
        self.pos = updated_state[0:2]
        self.vel = updated_state[2:4]
        self.missed_frames = 0
    
    def coast(self, dt):
        if not self.is_human: return
        updated_state = self.kf.predict(dt)
        self.pos = updated_state[0:2]
        self.vel = updated_state[2:4]


# --- HumanTracker Class ---
class HumanTracker:
    def __init__(self):
        rospy.loginfo("Starting Human Tracker Node (Kalman Filtered)...")
        self.enable_wall_tracks = rospy.get_param('~enable_wall_tracks', False)
        if sys.version_info.major < 3: rospy.logwarn("This script is intended for Python 3. You are using Python 2.")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.angle_rad = math.radians(GLOBAL_ANGLE)
        self.cos_a = math.cos(self.angle_rad)
        self.sin_a = math.sin(self.angle_rad)
        self.cloud_sub = rospy.Subscriber(
            "/velodyne_points_gpu", PointCloud2, self.cloud_callback, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/animated_human_tracks", 
            AnimatedMarkerArray, 
            queue_size=1
        )
        self.human_tracks = []
        self.wall_tracks = []
        self.last_time = None
        self.track_id_counter = 0
        self.current_sensor_pos = np.array([0.0, 0.0])
        if self.enable_wall_tracks:
            self.create_wall_tracks()

    def create_wall_tracks(self):
        rospy.loginfo("Generating static wall markers...")
        for y in np.arange(ROOM_Y_MIN, ROOM_Y_MAX, WALL_POINT_SPACING):
            self.wall_tracks.append(Track(self.track_id_counter, np.array([ROOM_X_MIN, y, 0]), False))
            self.track_id_counter += 1
        for y in np.arange(ROOM_Y_MIN, ROOM_Y_MAX, WALL_POINT_SPACING):
            self.wall_tracks.append(Track(self.track_id_counter, np.array([ROOM_X_MAX, y, 0]), False))
            self.track_id_counter += 1
        for x in np.arange(ROOM_X_MIN + WALL_POINT_SPACING, ROOM_X_MAX - WALL_POINT_SPACING, WALL_POINT_SPACING):
            self.wall_tracks.append(Track(self.track_id_counter, np.array([x, ROOM_Y_MIN, 0]), False))
            self.track_id_counter += 1
        for x in np.arange(ROOM_X_MIN + WALL_POINT_SPACING, ROOM_X_MAX - WALL_POINT_SPACING, WALL_POINT_SPACING):
            self.wall_tracks.append(Track(self.track_id_counter, np.array([x, ROOM_Y_MAX, 0]), False))
            self.track_id_counter += 1

    def apply_global_offset(self, x_map, y_map):
        x_shifted = x_map - GLOBAL_OFFSET_X
        y_shifted = y_map - GLOBAL_OFFSET_Y
        x_viz = x_shifted * self.cos_a - y_shifted * self.sin_a
        y_viz = x_shifted * self.sin_a + y_shifted * self.cos_a
        return x_viz, y_viz
        
    # SỬA LỖI TF: Dùng stamp của PointCloud2, tăng timeout, trả về None khi lỗi
    def transform_point_to_map(self, point_x, point_y, target_frame, source_frame, stamp):
        """Chuyển đổi vị trí từ frame tracking (base_footprint) sang frame hiển thị (map) 
           sử dụng đúng timestamp của PointCloud2 (stamp)."""
        try:
            p_source = PointStamped()
            p_source.header.frame_id = source_frame
            p_source.header.stamp = stamp
            p_source.point.x = point_x
            p_source.point.y = point_y
            
            # TĂNG THỜI GIAN TIMEOUT LÊN 1.0s để ổn định TF
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, rospy.Duration(1.0) 
            )
            p_target = tf_geo.do_transform_point(p_source, transform)
            return p_target.point.x, p_target.point.y
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # SỬA LỖI: Log warn và TRẢ VỀ None, None khi TF thất bại
            rospy.logwarn_throttle(1.0, f"TF Transform {source_frame} -> {target_frame} failed at {stamp.to_sec()}: {e}")
            return None, None 

    def cloud_callback(self, cloud_msg):
        # 1. Timing
        current_time = cloud_msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            self.publish_markers(cloud_msg.header) 
            return
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0:
            self.publish_markers(cloud_msg.header)
            return

        # 2-7. PC2 to Numpy, TF to FIXED_FRAME_ID, Filter, Downsample, DBSCAN (Giữ nguyên)
        try:
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)
            if not all(f in pc_data.dtype.names for f in ['x', 'y', 'z']): return
            xyz_points = np.stack([pc_data['x'], pc_data['y'], pc_data['z']], axis=1)
            mask = np.isfinite(xyz_points).all(axis=1)
            xyz_points = xyz_points[mask]
        except Exception as e:
            rospy.logerr(f"PC2 convert fail: {e}")
            return
        
        input_frame_id = cloud_msg.header.frame_id
        if input_frame_id != FIXED_FRAME_ID:
            try:
                transform = self.tf_buffer.lookup_transform(
                    FIXED_FRAME_ID, input_frame_id, cloud_msg.header.stamp, rospy.Duration(0.1)
                )
                t = transform.transform.translation
                self.current_sensor_pos = np.array([t.x, t.y])
                q = transform.transform.rotation
                T = quaternion_matrix([q.x, q.y, q.z, q.w])
                T[0:3, 3] = [t.x, t.y, t.z]
                homogeneous_points = np.hstack((xyz_points, np.ones((xyz_points.shape[0], 1))))
                xyz_points = np.dot(T, homogeneous_points.T).T[:, 0:3]
            except Exception:
                return
        else:
            self.current_sensor_pos = np.array([0.0, 0.0])

        filtered_points = xyz_points[xyz_points[:, 2] > GROUND_Z_MAX]
        if filtered_points.shape[0] == 0:
            self.update_human_tracks([], dt)
            self.publish_markers(cloud_msg.header)
            return

        quantized_cells = (filtered_points / DOWNSAMPLE_CELL_SIZE).astype(int)
        _, unique_indices = np.unique(quantized_cells, axis=0, return_index=True)
        downsampled_points = filtered_points[unique_indices]
        if downsampled_points.shape[0] < CLUSTER_MIN_SAMPLES:
            self.update_human_tracks([], dt)
            self.publish_markers(cloud_msg.header)
            return

        db = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit(downsampled_points[:, 0:2])
        downsampled_labels = db.labels_
        tree = KDTree(downsampled_points[:, 0:2])
        dists, indices = tree.query(filtered_points[:, 0:2], k=1)
        labels = downsampled_labels[indices]
        labels[dists > (DOWNSAMPLE_CELL_SIZE * 1.5)] = -1

        # 8. Filter Clusters (Lọc theo hình dạng và lọc tường tĩnh)
        unique_labels = set(labels)
        detections = [] 
        T = WALL_PROXIMITY_THRESHOLD

        for k in unique_labels:
            if k == -1: continue
            cluster_mask = (labels == k)
            cluster_points = filtered_points[cluster_mask]
            
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            centroid = np.mean(cluster_points, axis=0)
            height = max_pt[2] - min_pt[2]
            diameter = max(max_pt[0] - min_pt[0], max_pt[1] - min_pt[1])
            num_points = cluster_points.shape[0]

            # --- LỌC TƯỜNG BẰNG VÙNG ĐỆM ---
            if (centroid[0] <= ROOM_X_MIN + T) or (centroid[0] >= ROOM_X_MAX - T) or \
               (centroid[1] <= ROOM_Y_MIN + T) or (centroid[1] >= ROOM_Y_MAX - T):
                continue 
            
            dist_to_sensor = np.linalg.norm(centroid[0:2] - self.current_sensor_pos[0:2])
            is_valid = False

            # Check Close vs Far logic
            if dist_to_sensor < CLOSE_DETECTION_RADIUS:
                if (num_points >= CLOSE_MIN_POINTS) and (height > CLOSE_MIN_HEIGHT) and (diameter < HUMAN_MAX_DIAMETER):
                    is_valid = True
            else:
                if (HUMAN_MIN_POINTS < num_points < HUMAN_MAX_POINTS) and \
                (HUMAN_MIN_HEIGHT < height < HUMAN_MAX_HEIGHT) and \
                (diameter < HUMAN_MAX_DIAMETER):
                    is_valid = True

            if is_valid:
                is_too_close = False
                for existing_centroid in detections:
                    distance = np.linalg.norm(centroid - existing_centroid)
                    if distance < MIN_DETECTION_DISTANCE:
                        is_too_close = True
                        break
                    
                if not is_too_close:
                    detections.append(centroid)

        # 9. Tracking Update
        self.update_human_tracks(detections, dt)

        # 10. Visualize
        self.publish_markers(cloud_msg.header)

    def update_human_tracks(self, detections, dt):
        for track in self.human_tracks:
            track.coast(dt)
            
        if not self.human_tracks or not detections:
            if self.human_tracks:
                for track in self.human_tracks:
                    track.missed_frames += 1
            unmatched_detections = list(range(len(detections))) if detections else []
            
        else:
            track_positions = np.array([t.pos for t in self.human_tracks])
            detection_positions = np.array([d[0:2] for d in detections])
            cost_matrix = distance.cdist(track_positions, detection_positions)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            matched_track_indices = set()
            matched_detection_indices = set()

            for track_idx, det_idx in zip(row_ind, col_ind):
                cost = cost_matrix[track_idx, det_idx]
                
                if cost < TRACK_MAX_DIST:
                    track = self.human_tracks[track_idx]
                    detection = detections[det_idx]
                    track.update(detection, dt)
                    track.missed_frames = 0
                    matched_track_indices.add(track_idx)
                    matched_detection_indices.add(det_idx)

            unmatched_detections = [i for i in range(len(detections)) if i not in matched_detection_indices]
            for i, track in enumerate(self.human_tracks):
                if i not in matched_track_indices:
                    track.missed_frames += 1
                    
        self.human_tracks = [t for t in self.human_tracks if t.missed_frames <= TRACK_MAX_MISSES]

        for det_idx in unmatched_detections:
            new_track = Track(self.track_id_counter, detections[det_idx], is_human=True)
            self.human_tracks.append(new_track)
            self.track_id_counter += 1

    # SỬA TRIỆT ĐỂ: Đồng bộ thời gian và xử lý lỗi TF an toàn
    def publish_markers(self, header):
        array = AnimatedMarkerArray()
        
        # 1. LẤY TIMESTAMP CHÍNH XÁC CỦA ĐIỂM ĐÁM MÂY
        original_stamp = header.stamp 

        # 2. ĐẢM BẢO FRAME ID VÀ SỬ DỤNG TIMESTAMP CỦA ĐIỂM ĐÁM MÂY
        header.frame_id = TARGET_FRAME_ID
        header.stamp = original_stamp # <-- ĐÃ SỬA: SỬ DỤNG STAMP CỦA POINTCLOUD2

        if not self.human_tracks:
            self.marker_pub.publish(array)
            return

        for track in self.human_tracks:
            x_base, y_base = track.pos 
            vx, vy = track.vel
            
            # CHUYỂN ĐỔI TF VỊ TRÍ TỪ base_footprint -> map BẰNG original_stamp
            x_map, y_map = self.transform_point_to_map(
                x_base, y_base, 
                TARGET_FRAME_ID, 
                FIXED_FRAME_ID, 
                original_stamp
            )
            
            # Nếu TF thất bại, sử dụng vị trí map cuối cùng đã biết
            if x_map is None:
                x_map, y_map = track.pos_map_x, track.pos_map_y
                # Marker sẽ giữ vị trí cũ, không bị lệch
            else:
                # Cập nhật vị trí map frame chỉ khi TF thành công
                track.pos_map_x = x_map
                track.pos_map_y = y_map

            speed = np.linalg.norm(track.vel)
            
            if speed > 0.1:
                theta_deg = math.degrees(math.atan2(vy, vx))
            else:
                theta_deg = 0.0

            m = AnimatedMarker()
            m.header = header
            m.id = track.id
            m.ns = "animated_people"

            # --- CẤU HÌNH ANIMATED MARKER ---
            m.type = AnimatedMarker.MESH_RESOURCE
            m.mesh_resource = HUMAN_MESH_RESOURCE
            m.mesh_use_embedded_materials = True
            
            # Vị trí ĐÃ CHUYỂN ĐỔI (trong map frame)
            m.pose.position.x = x_map
            m.pose.position.y = y_map
            m.pose.position.z = 0.0 

            # Hướng (Roll = 90 độ, Yaw bù 90 độ)
            roll = math.radians(90.0)
            pitch = 0.0
            yaw = math.radians(theta_deg + 90.0)
            q = quaternion_from_euler(roll, pitch, yaw)

            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]

            # Scale 
            m.scale.x = HUMAN_MESH_SCALE
            m.scale.y = HUMAN_MESH_SCALE
            m.scale.z = HUMAN_MESH_SCALE

            # Animation Speed 
            m.animation_speed = min(1.0, 0.7 * speed)
            
            m.color.a = 1.0 
            m.color.r = track.color[0]
            m.color.g = track.color[1]
            m.color.b = track.color[2]
            
            m.lifetime = rospy.Duration(MARKER_LIFETIME) 

            array.markers.append(m)

        self.marker_pub.publish(array)


if __name__ == '__main__':
    try:
        rospy.init_node('human_tracker_adaptive', anonymous=True)
        tracker = HumanTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal Error: {e}")
