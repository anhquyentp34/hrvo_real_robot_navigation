#!/usr/bin/env python3
"""Spawn robot → unpause → load controllers (tuần tự, tránh Gazebo out / CM chưa sẵn sàng)."""
import subprocess
import sys
import time
import rospy
from gazebo_msgs.srv import SpawnModel, GetModelState
from std_srvs.srv import Empty


def wait_service(name, timeout):
    try:
        rospy.wait_for_service(name, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def main():
    rospy.init_node("gazebo_staged_bringup", anonymous=True)
    model = rospy.get_param("~model_name", "x_omni4wd")
    spawn_x = float(rospy.get_param("~x", -9.5))
    spawn_y = float(rospy.get_param("~y", -6.0))
    spawn_z = float(rospy.get_param("~z", 0.095))
    spawn_yaw = float(rospy.get_param("~yaw", 1.5708))
    cm_timeout = float(rospy.get_param("~controller_manager_timeout", 60.0))

    rospy.loginfo("=== Gazebo staged bringup: %s ===", model)

    if not wait_service("/gazebo/spawn_urdf_model", 60.0):
        rospy.logerr("Không có /gazebo/spawn_urdf_model")
        sys.exit(1)

    spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    req = SpawnModel._request_class()
    req.model_name = model
    req.model_xml = rospy.get_param("robot_description")
    req.robot_namespace = ""
    req.initial_pose.position.x = spawn_x
    req.initial_pose.position.y = spawn_y
    req.initial_pose.position.z = spawn_z
    import math
    req.initial_pose.orientation.z = math.sin(spawn_yaw * 0.5)
    req.initial_pose.orientation.w = math.cos(spawn_yaw * 0.5)
    req.reference_frame = "world"

    try:
        resp = spawn(req)
        if not resp.success:
            rospy.logerr("Spawn thất bại: %s", resp.status_message)
            sys.exit(2)
        rospy.loginfo("Spawn OK: %s", resp.status_message)
    except Exception as exc:
        rospy.logerr("Spawn exception: %s", exc)
        sys.exit(2)

    time.sleep(0.5)

    if wait_service("/gazebo/unpause_physics", 10.0):
        try:
            rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
            rospy.loginfo("Đã unpause Gazebo.")
        except Exception as exc:
            rospy.logwarn("unpause: %s", exc)

    # Chờ gazebo_ros_control khởi tạo controller_manager sau unpause
    time.sleep(3.0)

    cm_load = "/x_omni4wd/controller_manager/load_controller"
    if not wait_service(cm_load, cm_timeout):
        rospy.logerr("controller_manager không sẵn sàng sau %.0fs", cm_timeout)
        sys.exit(3)

    controllers = [
        "joint_state_controller",
        "front_left_wheel_velocity_controller",
        "rear_left_wheel_velocity_controller",
        "rear_right_wheel_velocity_controller",
        "front_right_wheel_velocity_controller",
    ]
    cmd = [
        "rosrun",
        "controller_manager",
        "spawner",
        "--namespace=/x_omni4wd",
    ] + controllers
    rospy.loginfo("Load controllers: %s", " ".join(controllers))
    ret = subprocess.call(cmd)
    if ret != 0:
        rospy.logerr("controller spawner exit %d", ret)
        sys.exit(ret)

    if wait_service("/gazebo/get_model_state", 5.0):
        state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        s = state(model, "")
        if s.success:
            p = s.pose.position
            rospy.loginfo("Robot ổn định tại (%.2f, %.2f, %.2f)", p.x, p.y, p.z)
    rospy.loginfo("=== Staged bringup hoàn tất ===")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
