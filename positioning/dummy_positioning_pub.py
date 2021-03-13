#!/usr/bin/env python

"""ROS node that performs 3D positioning on Marvelmind"""

import rospy
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from time import sleep, time, perf_counter
from sensor_msgs.msg import Imu
from marvelmind import MarvelmindHedge
import types

DRONE_READING = 303 #Reading from the drone yaw angle when facing anchor 10
BCN_OFFSET = None #Reference frame offset in rad

imu_data = None

def set_bcn_offset_from_yaw():
    global BCN_OFFSET
    drone_offset_deg = (360-DRONE_READING)
    BCN_OFFSET = drone_offset_deg * (np.pi/180)

def imu_data_callback(data):
    global imu_data
    imu_data = data

def send_vision_position_estimate(pub, pos):
    pub.publish(pos)

def positioning_pub():
    set_bcn_offset_from_yaw()
    print(BCN_OFFSET)
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.init_node('positioning_pub', anonymous=True)
    #Subscribe to get attitude
    rospy.Subscriber("/mavros/imu/data", Imu, imu_data_callback)

    pose = PoseStamped()

    rospy.sleep(1) # let data show up in subs
    while not rospy.is_shutdown():
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        temp_x = 0.1
        temp_y = -2.1
        pose.pose.position.y = (temp_x*np.cos(BCN_OFFSET)-temp_y*np.sin(BCN_OFFSET))
        pose.pose.position.x = -(temp_x*np.sin(BCN_OFFSET)+temp_y*np.cos(BCN_OFFSET))
        pose.pose.position.z = -1.1

        pose.pose.orientation = Quaternion(
        	imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w)
        pose_pub.publish(pose)

if __name__ == "__main__":
    positioning_pub()

