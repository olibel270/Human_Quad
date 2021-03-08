#!/usr/bin/env python

"""ROS node that performs 3D positioning on Marvelmind"""

import rospy
import numpy as np
import sys
import configparser
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from time import sleep, time, perf_counter
from sensor_msgs.msg import Imu
from marvelmind import MarvelmindHedge
import types

DRONE_READING = 300 #Reading from the drone yaw angle when facing anchor 10
BCN_OFFSET = None #Reference frame offset in rad

imu_data = None

def imu_data_callback(data):
    global imu_data
    imu_data = data

def send_vision_position_estimate(pub, pos):
    pub.publish(pos)

def positioning_pub():
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.init_node('positioning_pub', anonymous=True)
    #Subscribe to get attitude
    rospy.Subscriber("/mavros/imu/data", Imu, imu_data_callback)

    pose = PoseStamped()

    rospy.sleep(1) # let data show up in subs
    while not rospy.is_shutdown():
        try:
            position = [0,1.5,-2,1] 
            sleep(0.01)
            pose.header = Header()
            pose.header.stamp = rospy.Time.now()
            temp_x = position[1]
            temp_y = position[2]
            pose.pose.position.x = (temp_x*np.cos(BCN_OFFSET)-temp_y*np.sin(BCN_OFFSET))
            pose.pose.position.y = -(temp_x*np.sin(BCN_OFFSET)+temp_y*np.cos(BCN_OFFSET))
            pose.pose.position.z = -position[3]

            pose.pose.orientation = Quaternion(
        	imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w)
            pose_pub.publish(pose)
            print(pose)
        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()

if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('../config.ini')
    BCN_OFFSET = float(config['hardware_setup']['local_ned_offset_rad'])
    print(BCN_OFFSET)
    positioning_pub()
