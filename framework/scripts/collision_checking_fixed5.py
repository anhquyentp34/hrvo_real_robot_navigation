#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

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

        distance1 = Float64()
        distance1.data = distance(teb_point.pose.position, predicted_object_position)
        distance_pub.publish(distance1)

        if distance(teb_point.pose.position, predicted_object_position) < distance_threshold:
            return True, predicted_object_path, predicted_object_position

    return False, predicted_object_path, None

def teb_feedback_callback(feedback_msg, object_name, distance_threshold):
    global object_positions, object_velocities, collision_point_pub, predicted_object_path_pub

    selected_traj = feedback_msg.trajectories[feedback_msg.selected_trajectory_idx]
    teb_traj_points = [point for point in selected_traj.trajectory]

    collision_predicted, predicted_object_path, collision_point = predict_collision(teb_traj_points, object_positions[object_name], object_velocities[object_name], time_horizon=5.0, distance_threshold=distance_threshold)

    predicted_object_path_pub.publish(predicted_object_path)

    if collision_predicted:
        rospy.logwarn(f"Collision predicted between the robot and {object_name}!")
        collision_point_stamped = PointStamped()
        collision_point_stamped.header.frame_id = "map"
        collision_point_stamped.header.stamp = rospy.Time.now()
        collision_point_stamped.point = collision_point
        collision_point_pub[object_name].publish(collision_point_stamped)

def actor_pose_callback(msg, object_name):
    global object_positions, object_velocities, object_marker_pub

    object_positions[object_name] = msg.pose.position

    # Create and publish the object marker
    object_marker = Marker()
    object_marker.header.frame_id = "map"
    object_marker.header.stamp = rospy.Time.now()
    object_marker.ns = object_name
    object_marker.id = 0
    object_marker.type = Marker.SPHERE
    object_marker.action = Marker.ADD
    object_marker.pose.position = object_positions[object_name]
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

    object_marker_pub[object_name].publish(object_marker)

def actor_velocity_callback(msg, object_name):
    global object_velocities
    object_velocities[object_name] = msg.linear

 rospy.init_node("update_model_state")
   #rospy.init_node("collision_prediction")

object_names = ["actor1", "actor2"]
time_horizon = rospy.get_param("~time_horizon", 5.0)
distance_threshold = rospy.get_param("~distance_threshold", 1.5)

object_positions = {}
object_velocities = {}

collision_point_pub =n
predicted_object_path_pub = {}
object_marker_pub = {}
distance_pub = {}

for object_name in object_names:
    rospy.Subscriber(f"/{object_name}/actor_pose", PoseStamped, actor_pose_callback, object_name)
    rospy.Subscriber(f"/{object_name}/actor_velocity", Twist, actor_velocity_callback, object_name)
    rospy.Subscriber(f"/move_base/TebLocalPlannerROS/teb_feedback_{object_name}", FeedbackMsg, teb_feedback_callback, object_name)

    collision_point_pub[object_name] = rospy.Publisher(f"/predicted_collision_point_{object_name}", PointStamped, queue_size=1)
    predicted_object_path_pub[object_name] = rospy.Publisher(f"/predicted_object_path_{object_name}", Path, queue_size=1)
    object_marker_pub[object_name] = rospy.Publisher(f"/object_marker_{object_name}", Marker, queue_size=10)
    distance_pub[object_name] = rospy.Publisher(f"/distance_{object_name}", Float64, queue_size=10)

rospy.spin()

