#!/usr/bin/env python
"""ROS node that performs 3D positioning on Marvelmind"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from time import sleep, time, perf_counter
from sensor_msgs.msg import Imu

import types
BCN_OFFSET= 2.70526 #Reference frame offset in rad

class filo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(len(buf))

def send_vision_position_estimate(pub, pos):
    pub.publish(pos)

def positioning_pub():
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=5)
    rospy.init_node('positioning_pub', anonymous=True)
    pose = PoseStamped()

    f = filo()
    rospy.sleep(1) # let data show up in subs
    while mavlink_pub.get_num_connections() <= 0:
        pass
    while not rospy.is_shutdown():
        #get coordinates
      
        temp_x = coords.x/1000
        temp_y = coords.y/1000
        pose.position.x = (temp_x*np.cos(BCN_OFFSET)+temp_y*np.sin(BCN_OFFSET))
        pose.position.y = -(-temp_x*np.cos(BCN_OFFSET)+temp_y*np.sin(BCN_OFFSET))
        
        pose_pub.publish(pose)


if __name__ == "__main__":
    positioning_pub()
