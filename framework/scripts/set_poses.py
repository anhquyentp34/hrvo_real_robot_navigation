#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def set_model_pose(model_name, x, y, z, roll, pitch, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]

        result = set_state(state)
        if result.success:
            rospy.loginfo("Model pose set successfully: %s", model_name)
        else:
            rospy.logerr("Failed to set model pose: %s", model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('set_model_pose_node')

    import tf

    # Set pose for robot_base model
    set_model_pose('robot_base', x=0, y=5, z=1, roll=0, pitch=0, yaw=-3.14159/4)
    # set_model_pose('robot_base', x=0, y=0, z=1, roll=0, pitch=0, yaw=0)

    # Set pose for actor3 model
   # set_model_pose('actor3', x=4, y=5, z=1, roll=0, pitch=0, yaw=0)
