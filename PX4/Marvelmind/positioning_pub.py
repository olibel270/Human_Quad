#!/usr/bin/env python
"""ROS node that performs 3D positioning on Marvelmind"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from time import sleep, time, perf_counter
from sensor_msgs.msg import Imu
from marvelmind import MarvelmindHedge
import types

BCN_OFFSET= 2.70526 #Reference frame offset in rad

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

    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    hedge.start()

    rospy.sleep(1) # let data show up in subs
    while mavlink_pub.get_num_connections() <= 0:
        pass
    while not rospy.is_shutdown():
        try:
            hedge.dataEvent.wait()
            hedge.dataEvent.clear()#get coordinates

            if (hedge.positionUpdated):
                print(hedge.position)

        temp_x = hedge.position[1]
        temp_y = hedge.position[2]
        pose.position.x = (temp_x*np.cos(BCN_OFFSET)+temp_y*np.sin(BCN_OFFSET))
        pose.position.y = -(-temp_x*np.cos(BCN_OFFSET)+temp_y*np.sin(BCN_OFFSET))
        pose.position.z = -hedge.position[3]

        pose.orentation = Quaternion(
        	imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w)
        pose_pub.publish(pose)


if __name__ == "__main__":
    positioning_pub()
