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

DRONE_READING = 310 #Reading from the drone yaw angle when facing anchor 10
BCN_OFFSET = None #Reference frame offset in rad

imu_data = None

class SingletonMeta(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(SingletonMeta,cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class PositionPub(metaclass=SingletonMeta):

    def __init__(self):
        self.hedge = None

    def set_bcn_offset_from_yaw(self):
        global BCN_OFFSET
        drone_offset_deg = (360-DRONE_READING)
        BCN_OFFSET = drone_offset_deg * (np.pi/180)
    
    def imu_data_callback(self, data):
        global imu_data
        imu_data = data
    
    def send_vision_position_estimate(self, pub, pos):
        pub.publish(pos)
    
    def positioning_pub(self):
        self.set_bcn_offset_from_yaw()
        print(BCN_OFFSET)
        pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        rospy.init_node('positioning_pub', anonymous=True)
        #Subscribe to get attitude
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_data_callback)
    
        pose = PoseStamped()
    
        self.hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
        if (len(sys.argv)>1):
            self.hedge.tty= sys.argv[1]
        self.hedge.start()
    
        rospy.sleep(1) # let data show up in subs
        while not rospy.is_shutdown():
            try:
                self.hedge.dataEvent.wait()
                self.hedge.dataEvent.clear()#get coordinates
    
                if (self.hedge.positionUpdated):
                    position = self.hedge.position()
                else:
                    continue
    
                pose.header = Header()
                pose.header.stamp = rospy.Time.now()
                temp_x = position[1]
                temp_y = position[2]
                pose.pose.position.y = (temp_x*np.cos(BCN_OFFSET)-temp_y*np.sin(BCN_OFFSET))
                pose.pose.position.x = -(temp_x*np.sin(BCN_OFFSET)+temp_y*np.cos(BCN_OFFSET))
                pose.pose.position.z = -position[3]
    
                pose.pose.orientation = Quaternion(
            	imu_data.orientation.x,
                    imu_data.orientation.y,
                    imu_data.orientation.z,
                    imu_data.orientation.w)
                pose_pub.publish(pose)
    
            except KeyboardInterrupt:
                self.hedge.stop() #Close serial port
                sys.exit()
    
if __name__ == "__main__":
    position_pub = PositionPub()
    position_pub.positioning_pub()

