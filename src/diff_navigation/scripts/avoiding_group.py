#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import OccupancyGrid, MapMetaData
from gazebo_msgs.msg import ModelStates
import numpy as np

ACTOR_A = "actor32"
ACTOR_B = "actor33"

actor = {
    ACTOR_A: None,  # (x,y,vx,vy)
    ACTOR_B: None
}

def on_states(msg):
    for i, name in enumerate(msg.name):
        if name in actor:
            p = msg.pose[i].position
            v = msg.twist[i].linear
            actor[name] = (p.x, p.y, v.x, v.y)


def compute_group_ellipse():
    A = actor[ACTOR_A]
    B = actor[ACTOR_B]
    if A is None or B is None:
        return None

    xA,yA,vxA,vyA = A
    xB,yB,vxB,vyB = B

    # center
    Cx = (xA + xB) / 2.0
    Cy = (yA + yB) / 2.0

    # orientation
    vxG = vxA + vxB
    vyG = vyA + vyB
    if abs(vxG)+abs(vyG) < 1e-3:
        yaw = 0.0
    else:
        yaw = math.atan2(vyG, vxG)

    # ellipse axes
    dist = math.hypot(xA - xB, yA - yB)
    b = dist/2.0 + 0.2  # semi-minor
    a = 1.0             # semi-major

    return (Cx, Cy, a, b, yaw)


def main():
    rospy.init_node("proxemic_costmap")

    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, on_states)
    pub = rospy.Publisher("/proxemic_costmap", OccupancyGrid, queue_size=1, latch=True)

    # map params
    RES = 0.05   # 5cm
    WIDTH = 160  # 8m
    HEIGHT = 160 # 8m

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        ellipse = compute_group_ellipse()
        if ellipse is None:
            rate.sleep()
            continue

        Cx, Cy, a, b, yaw = ellipse

        # OccupancyGrid init
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"

        meta = MapMetaData()
        meta.resolution = RES
        meta.width = WIDTH
        meta.height = HEIGHT
        meta.origin.position.x = Cx - (WIDTH*RES)/2.0
        meta.origin.position.y = Cy - (HEIGHT*RES)/2.0
        meta.origin.position.z = 0.0

        grid.info = meta

        data = np.zeros((HEIGHT, WIDTH), dtype=np.int8)

        # cos-yaw
        cosT = math.cos(yaw)
        sinT = math.sin(yaw)

        # Gaussian sigma
        sigma = 0.3

        for j in range(HEIGHT):
            for i in range(WIDTH):

                # world coords of cell center
                wx = meta.origin.position.x + (i + 0.5) * RES
                wy = meta.origin.position.y + (j + 0.5) * RES

                # transform into ellipse frame
                dx = wx - Cx
                dy = wy - Cy

                x_prime =  dx*cosT + dy*sinT
                y_prime = -dx*sinT + dy*cosT

                # normalized ellipse equation
                E = (x_prime/a)**2 + (y_prime/b)**2

                # Gaussian cost
                cost = math.exp(-E / (2 * sigma * sigma)) * 100.0

                if cost < 1:
                    cost = 0

                data[j,i] = int(min(cost, 100))

        grid.data = data.flatten().tolist()
        pub.publish(grid)

        rate.sleep()


if __name__ == "__main__":
    main()

