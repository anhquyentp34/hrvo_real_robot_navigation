#!/usr/bin/env python

import rospy

from ros_pub.msg import info
from std_msgs.msg import String


def pub():
    pub = rospy.Publisher('topic_info', info, queue_size=10)
    rospy.init_node('pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = info
    while not rospy.is_shutdown():
        msg.name = 'nam'
        msg.age = 20
        
        pub.publish(msg)
        rospy.loginfo("sucess")
        rate.sleep()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass
