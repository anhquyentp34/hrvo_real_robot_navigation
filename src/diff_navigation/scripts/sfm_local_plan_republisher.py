#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from animated_marker_msgs.msg import AnimatedMarkerArray
from animated_marker_tutorial import createAnimatedPersonMarker
import math

actor_prefix = "actor"

# ===============================
# DANH SÁCH ACTOR TĨNH & ĐỘNG
# ===============================
static_actors = ["actor31", "actor32", "actor33"]
dynamic_actors = ["actor1", "actor21", "actor22"]
# ===============================

colors = [(0.2,0.8,0.8), (0.8,0.2,0.2), (0.8,0.8,0.2), (0.2,0.8,0.2)]
poses = {}

def cb_states(msg):
    for i, name in enumerate(msg.name):
        if name.lower().startswith(actor_prefix):
            p = msg.pose[i].position
            o = msg.pose[i].orientation
            poses[name] = (p.x, p.y, p.z, o.z, o.w)

def main():
    rospy.init_node("actors_to_animated_markers")
    rospy.Subscriber("/gazebo/model_states", ModelStates, cb_states, queue_size=1)
    pub = rospy.Publisher("/animated_markers", AnimatedMarkerArray, queue_size=1)

    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        array = AnimatedMarkerArray()

        for idx, (name, (x, y, z, oz, ow)) in enumerate(sorted(poses.items())):
            
            # --- Phân loại actor theo tên ---
            if name in static_actors:
                animSpeed = 0.0          # Đứng im
            else:
                animSpeed = 1.0          # Chuyển động

            # Tính góc quay
            theta = 2 * math.atan2(oz, ow) * 180 / math.pi

            # Màu
            c = colors[idx % len(colors)]

            # Tạo marker
            array.markers.append(
                createAnimatedPersonMarker(
                    idx,
                    pos=(x, y),
                    thetaDeg=theta,
                    color=c,
                    animationSpeed=animSpeed
                )
            )

        pub.publish(array)
        rate.sleep()

if __name__ == "__main__":
    main()

