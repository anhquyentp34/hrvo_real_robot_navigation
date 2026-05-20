#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

actor_data = []
robot_data = []

def actor_callback(msg):
    global actor_data
    x = msg.linear.x
    y = msg.linear.y
    actor_data.append((rospy.Time.now().to_sec(), x, y))

def robot_callback(msg):
    global robot_data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    robot_data.append((rospy.Time.now().to_sec(), x, y))

def save_data_to_csv(filename, data):
    with open(filename, 'w') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['time', 'x', 'y'])
        for row in data:
            csv_writer.writerow(row)

def main():
    rospy.init_node('collect_trajectory_data', anonymous=True)
    rospy.Subscriber('/actor_velocity', Twist, actor_callback)
    rospy.Subscriber('/servicebot/odom', Odometry, robot_callback)

    rospy.loginfo("Collecting trajectory data...")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Saving trajectory data to CSV files...")
    save_data_to_csv('actor_trajectory.csv', actor_data)
    save_data_to_csv('robot_trajectory.csv', robot_data)
    rospy.loginfo("Done.")

if __name__ == '__main__':
    main()
