#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
msg = """
    *** CAFE - SERVICE ***
Where do you want the robot to go?
    ---------------------
    0 = Waitting Room (Defaut)
    1 = Room 1
    2 = Room 2
    3 = Room 3
    4 = Room 4
    5 = Waitting Room--> Room 1--> Room 2--> Room 3--> Room 4--> Waitting Room
Enter a number ?

"""
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)
def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def movebase_client(x, y, z, x1, y1, z1,  w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = x1
    goal.target_pose.pose.orientation.y= y1
    goal.target_pose.pose.orientation.z = z1
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    

if __name__ == '__main__':
    settings = saveTerminalSettings()
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    try:
        print(msg)
        while(1):
            
            key = getKey(settings, key_timeout)
            if key == "0":
                print("Moving to Waitting Room ...")
                x = 0.941516876221
                y = -0.0630865097046
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.0
                w = 1.0
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Waitting Room !!!")
                    print(msg)

            elif key == "1":
                print("Moving to Room 1...")
                x = 7.09024906158
                y = 9.7411403656
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.0176215419946
                w = 0.999844728574
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 1!!!")
                    print(msg)
            elif key == "2":
                print("Moving to Room 2...")
                x = 9.94739532471
                y = -1.69266545773
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = -0.681782535806
                w = 0.731554901473
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 2 !!!")
                    print(msg)
            elif key == "3":
                print("Moving to Room 3 ...")
                x = -1.96298027039
                y = -6.99760770798
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.999999804557
                w = 0.000625208971573
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room  3 !!!")
                    print(msg)
            elif key == "4":
                print("Moving to Room 4...")
                x = -3.94347667694
                y = 9.01787567139
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.703679422764
                w = 0.710517607086
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 4 !!!")
                    print(msg)
            elif key == "5":
                print("Waitting Room--> Room 1--> Room 2--> Room 3--> Room 4--> Waitting Room ...")
                print("Moving to Waitting Room ...")
                x = 0.941516876221
                y = -0.0630865097046
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.0
                w = 1.0
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Waitting Room !!!")
                print("Moving to Room 1...")
                x = 7.09024906158
                y = 9.7411403656
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.0176215419946
                w = 0.999844728574
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 1!!!")
                  
                print("Moving to Room 2...")
                x = 9.94739532471
                y = -1.69266545773
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = -0.681782535806
                w = 0.731554901473
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 2 !!!")
                    
                print("Moving to Room 3 ...")
                x = -1.96298027039
                y = -6.99760770798
                z = 0.0
                x1 = 0.0
                y1 = 0.0
                z1 = 0.999999804557
                w = 0.000625208971573
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room  3 !!!")

                print("Moving to Room 4...")
                x = -5.01445627213
                y = 8.54764842987
                w = 1.0
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Room 4 !!!")

                print("Moving to Waitting Room ...")
                x = 0.941516876221
                y = -0.0630865097046
                w = 1.0
                rospy.init_node('movebase_client_py')
                result = movebase_client(x, y, z, x1, y1, z1,  w)
                if result:
                    rospy.loginfo("Moved to Waitting Room !!!")
                    print(msg)

            else:
                if key == '':
                    continue
                elif (key == '\x03'):
                    break
                else: 
                    print("Number selected not in list. Try again ")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")





