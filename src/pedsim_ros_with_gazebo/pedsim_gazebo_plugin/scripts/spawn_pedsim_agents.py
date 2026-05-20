#!/usr/bin/env python3
"""
Created on Mon Dec  2 17:03:34 2019

@author: quyenanh pt
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates

# Thời gian nghỉ giữa mỗi lần spawn (giảm tải gzserver khi đám đông lớn).
SPAWN_DELAY_SEC = 0.05

spawned_ids = set()
_gave_up = False


def _actor_model_name(actor_id):
    return "actor{}".format(actor_id)


def actor_poses_callback(actors):
    global _gave_up
    if _gave_up:
        return
    if not actors.agent_states:
        return
    for actor in actors.agent_states:
        actor_id = str(actor.id)
        model_name = _actor_model_name(actor_id)
        if model_name in spawned_ids:
            continue
        actor_pose = actor.pose
        rospy.loginfo("Spawning model: actor_id = %s, model_name = %s", actor_id, model_name)

        model_pose = Pose(Point(x=actor_pose.position.x,
                               y=actor_pose.position.y,
                               z=actor_pose.position.z),
                         Quaternion(actor_pose.orientation.x,
                                    actor_pose.orientation.y,
                                    actor_pose.orientation.z,
                                    actor_pose.orientation.w))

        try:
            spawn_model(model_name, xml_string, "", model_pose, "world")
            spawned_ids.add(model_name)
            if SPAWN_DELAY_SEC > 0.0:
                rospy.sleep(SPAWN_DELAY_SEC)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(
                10.0,
                "spawn_sdf_model failed (gzserver có thể đã crash): %s — ngắt spawn actors.",
                e,
            )
            _gave_up = True
            return
    if not _gave_up:
        rospy.signal_shutdown("all agents have been spawned !")




if __name__ == '__main__':

    rospy.init_node("spawn_pedsim_agents")

    # Cập nhật delay từ param (code `if __name__` thuộc module scope, không cần global).
    SPAWN_DELAY_SEC = float(rospy.get_param("~spawn_delay_sec", 0.05))

    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model_simple.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback, queue_size=1)

    rospy.spin()
