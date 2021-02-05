#!/usr/bin/env python
"""ROS node that tests offboard mode"""

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

current_pose = PoseStamped()

def pose_data_callback(pose):
    global current_pose
    current_pose = pose

def offboard_test():
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    rospy.init_node('offboard_test', anonymous=True)
    #Subscribe to get current position
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_data_callback)

    setpoint = PoseStamped()

    setpoint.header = Header()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.x = current_pose.pose.position.x 
    setpoint.pose.position.y = current_pose.pose.position.y
    setpoint.pose.position.z = current_pose.pose.position.z-1

    while True:
        setpoint_pub.publish(setpoint)


if __name__ == "__main__":
    offboard_test()
