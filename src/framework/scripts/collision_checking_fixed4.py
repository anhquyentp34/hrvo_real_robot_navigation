#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Twist, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

def distance(point1, point2):
    return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

def predict_collision(teb_traj_points, object_position, object_velocity, time_horizon, distance_threshold):
    predicted_object_path = Path()
    predicted_object_path.header.stamp = rospy.Time.now()
    predicted_object_path.header.frame_id = "map"

    for teb_point in teb_traj_points:
        time_from_start = teb_point.time_from_start.to_sec()
        if time_from_start > time_horizon:
            break

        predicted_object_position = Point()
        predicted_object_position.x = object_position.x + object_velocity.x * time_from_start
        predicted_object_position.y = object_position.y + object_velocity.y * time_from_start
        predicted_object_position.z = object_position.z + object_velocity.z * time_from_start

        predicted_pose = PoseStamped()
        predicted_pose.header.stamp = rospy.Time.now()
        predicted_pose.header.frame_id = "map"
        predicted_pose.pose.position = predicted_object_position
        predicted_object_path.poses.append(predicted_pose)

        if distance(teb_point.pose.position, predicted_object_position) < distance_threshold:
            return True, predicted_object_path, predicted_object_position

    return False, predicted_object_path, None

def teb_feedback_callback(feedback_msg):
    global object_position, object_velocity, collision_point_pub, predicted_object_path_pub

    selected_traj = feedback_msg.trajectories[feedback_msg.selected_trajectory_idx]
    teb_traj_points = [point for point in selected_traj.trajectory]

    collision_predicted, predicted_object_path, collision_point = predict_collision(teb_traj_points, object_position, object_velocity, time_horizon=5.0, distance_threshold=3)

    predicted_object_path_pub.publish(predicted_object_path)

    if collision_predicted:
        rospy.logwarn("Collision predicted between the robot and the object!")
        collision_point_stamped = PointStamped()
        collision_point_stamped.header.frame_id = "map"
        collision_point_stamped.header.stamp = rospy.Time.now()
        collision_point_stamped.point = collision_point
        collision_point_pub.publish(collision_point_stamped)

def actor_pose_callback(msg):
    global object_position, object_velocity, object_name, object_marker_pub

    object_position = msg.pose.position
    create_and_publish_object_marker(object_position, object_marker_pub)

def create_and_publish_object_marker(position, publisher):
    object_marker = Marker()
    object_marker.header.frame_id = "map"
    object_marker.header.stamp = rospy.Time.now()
    object_marker.ns = "object"
    object_marker.id = 0
    object_marker.type = Marker.SPHERE
    object_marker.action = Marker.ADD
    object_marker.pose.position = position
    object_marker.pose.orientation.x = 0.0
    object_marker.pose.orientation.y = 0.0
    object_marker.pose.orientation.z = 0.0
    object_marker.pose.orientation.w = 1.0
    object_marker.scale.x = 1.0
    object_marker.scale.y = 1.0
    object_marker.scale.z = 1.0
    object_marker.color.r = 0.0
    object_marker.color.g = 1.0
    object_marker.color.b = 0.0
    object_marker.color.a = 1.0
    object_marker.lifetime = rospy.Duration(0)

def actor_velocity_callback(msg):
    global object_velocity
    object_velocity = msg.linear

rospy.init_node("collision_prediction")

object_name = rospy.get_param("~object_name", "actor1")
time_horizon = rospy.get_param("~time_horizon", 5.0)
distance_threshold = rospy.get_param("~distance_threshold", 3)

object_position = None
object_velocity = None

rospy.Subscriber("/actor_velocity", Twist, actor_velocity_callback)
rospy.Subscriber("/actor_pose", PoseStamped, actor_pose_callback)
rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, teb_feedback_callback)

collision_point_pub = rospy.Publisher("/predicted_collision_point", PointStamped, queue_size=1)
predicted_object_path_pub = rospy.Publisher("/predicted_object_path", Path, queue_size=1)
object_marker_pub = rospy.Publisher("/object_marker", Marker, queue_size=10)

rospy.spin()
