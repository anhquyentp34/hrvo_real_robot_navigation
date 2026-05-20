#!/usr/bin/env python3
import rospy
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def debug_topics():
    rospy.init_node('debug_topics_node')
    
    print("=== DEBUG TOPICS ===")
    print("Waiting for topics to be available...")
    
    # Wait for topics to be available
    time.sleep(2)
    
    # Check if topics exist
    topics = [
        '/cmd_path_1',
        '/cmd_path_2', 
        '/cmd_path_3',
        '/cmd_path_4'
    ]
    
    for topic in topics:
        try:
            # Try to get a message from the topic
            msg = rospy.wait_for_message(topic, Path, timeout=5.0)
            print(f"✅ {topic}: {len(msg.poses)} poses")
            
            # Print first pose
            if len(msg.poses) > 0:
                first_pose = msg.poses[0]
                print(f"   First pose: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f})")
                
        except rospy.ROSException as e:
            print(f"❌ {topic}: {e}")
    
    print("\n=== ACTOR POSITIONS ===")
    print("Check Gazebo GUI to see if actors are moving from their initial positions:")
    print("- Doctor 1: (-5, 5) - Should move in square pattern")
    print("- Doctor 2: (5, 5) - Should move in circle pattern") 
    print("- Doctor 3: (-5, -5) - Should move in straight line")
    print("- Doctor 4: (5, -5) - Should move in triangle pattern")
    
    print("\n=== EXPECTED BEHAVIOR ===")
    print("Actors should start moving immediately from their initial positions")
    print("They should NOT go to (0,0) first!")
    
    # Keep running to monitor
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        debug_topics()
    except rospy.ROSInterruptException:
        pass 