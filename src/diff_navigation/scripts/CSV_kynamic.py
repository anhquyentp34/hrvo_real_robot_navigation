#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
import csv
import os
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class RobotDataLogger:
    def __init__(self):
        rospy.init_node("robot_data_logger", anonymous=True)

        # DUONG DAN VA TEN FILE LUU
        self.log_file_path = "/home/hnam/nam_xp_ws/kinamonic.csv"
        
        # Bien luu tru du lieu
        self.cmd_vel_data = None
        self.odom_data = None
        self.sfm_data = None
        
        # Bien trang thai de dam bao ghi header dung cach
        self.header_written = False
        
        # Khoi tao file log: Xoa du lieu cu
        if os.path.exists(self.log_file_path):
            os.remove(self.log_file_path)
            rospy.loginfo(f"Da xoa file log cu: {self.log_file_path}")

        # THIET LAP SUBSCRIBERS
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/sfm/velocity", Vector3, self.sfm_vel_cb)
        
        # Thiet lap tan so ghi du lieu (Hz)
        self.rate = rospy.Rate(50) # Ghi 50 lan/giay

        rospy.loginfo("Robot Data Logger SAN SANG. Bat dau ghi du lieu.")
        self.run_logger()

    def cmd_vel_cb(self, msg):
        # Luu tru lenh van toc cuoi cung (sau khi co rang buoc)
        self.cmd_vel_data = {'v_cmd': msg.linear.x, 'w_cmd': msg.angular.z}

    def odom_cb(self, msg):
        # Luu tru van toc thuc te (v_act, w_act) tu Odom
        self.odom_data = {'v_act': msg.twist.twist.linear.x, 
                          'w_act': msg.twist.twist.angular.z}

    def sfm_vel_cb(self, msg):
        # Luu tru van toc mong muon tu SFM (vx_sfm, vy_sfm)
        self.sfm_data = {'vx_sfm': msg.x, 'vy_sfm': msg.y}

    def run_logger(self):
        """Vong lap chinh de ghi du lieu vao file."""
        
        # Dinh nghia cac cot can luu
        self.fieldnames = [
            'time', 'v_cmd', 'w_cmd', 'v_act', 'w_act', 'vx_sfm', 'vy_sfm'
        ]

        with open(self.log_file_path, 'w') as csvfile:
            self.writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)

            while not rospy.is_shutdown():
                # Chi ghi du lieu khi da nhan duoc tat ca cac thong diep it nhat mot lan
                if self.cmd_vel_data and self.odom_data and self.sfm_data:
                    if not self.header_written:
                        self.writer.writeheader()
                        self.header_written = True

                    # Tong hop du lieu
                    log_entry = {
                        'time': rospy.get_time(),
                        **self.cmd_vel_data,
                        **self.odom_data,
                        **self.sfm_data
                    }
                    
                    try:
                        self.writer.writerow(log_entry)
                    except Exception as e:
                        rospy.logwarn(f"Loi khi ghi du lieu: {e}")
                        
                self.rate.sleep()

if __name__ == "__main__":
    try:
        RobotDataLogger()
    except rospy.ROSInterruptException:
        pass
