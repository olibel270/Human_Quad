#!/usr/bin/env python
"""ROS node that tests offboard mode"""

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

current_pose = None

def pose_data_callback(pose):
    global current_pose
    current_pose = pose

def offboard_test():
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    rospy.init_node('offboard_test', anonymous=True)
    #Subscribe to get current position
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_data_callback)
    rospy.sleep(1)
    r = rospy.Rate(5)
    setpoint = PoseStamped()

    setpoint.header = Header()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.x = current_pose.pose.position.x 
    setpoint.pose.position.y = current_pose.pose.position.y
    setpoint.pose.position.z = current_pose.pose.position.z+1
    setpoint.pose.orientation = current_pose.pose.orientation
    
    count = 0
    while True:
        setpoint_pub.publish(setpoint)
        if(count==75):
            setpoint.pose.position.z += 2
        if(count==100):
            setpoint.pose.position.y -= 2
        r.sleep()
        count +=1


if __name__ == "__main__":
    offboard_test()
