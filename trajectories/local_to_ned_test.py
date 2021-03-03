#!/usr/bin/env python
"""ROS node that tests local frame setpoint navigation """
"""IMPORTANT!!!!!! CHECK THE SETPOINT COORDINATES ARE RIGHT BEFORE FLYING!!!!!!!!!!"""
"""The flying portion was commented out. Uncomment when convinced of transformation accuracy"""

import configparser
import rospy
import sys, threading
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Header
from transform_functions import local_to_ned

current_pose = PoseStamped()
MAP_TO_DRONE_RAD = 0
FLY_TIME = 24 #24s total fly time for the test. Should be setpoint distance based, but good for now + it's just a test
SETPOINT_FREQ = 5

def pose_data_callback(pose):
    global current_pose
    current_pose = pose

def publisher_thread(setpoints, publisher, rate, fly_time=0):
    r = rospy.Rate(rate)
    setpoint = setpoints[0]
    if fly_time != 0:
        iterations = fly_time / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    else:
        iterations = 5 / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    count = 0
    while count<iterations:
        publisher.publish(setpoint)
        if(count==iterations*(1/6)):#4s
            setpoint = setpoints[1]
            print("Performing SQUARE!")
            print("To Setpoint 1")
        if(count==iterations*(2/6)):#8s
            setpoint = setpoints[2] 
            print("To Setpoint 2")
        if(count==iterations*(3/6)):#12s
            setpoint = setpoints[3]
            print("To Setpoint 3")
        if(count==iterations*(4/6)):#16s
            setpoint = setpoints[4]
            print("To Setpoint 4")
        if(count==iterations*(5/6)):#20s
            setpoint = setpoints[5]
            print("To Setpoint 1")
        r.sleep()
        count += 1

def define_drone_setpoints(local_setpoints):
    # Define local setpoint coordinates
    local_setpoint_coords = np.array([[0,-2,1,0],[1.5,-2,1,0],[1.5,-0.5,1,0],[0,-0.5,1,0],[0,-2,1,0]])
    # Get equivalent NED setpoint coordinates
    setpoints_ned_coords = local_to_ned(local_setpoint_coords, MAP_TO_DRONE_RAD)
    # assign to setpoint objects 
    tmp = []
    for i,setpoint in enumerate(local_setpoints):
        if (i==0):
            tmp.append(local_setpoints[i])
            continue
        tmp.append(PoseStamped())
        tmp[i].pose.position.x = setpoints_ned_coords[i-1][0]
        tmp[i].pose.position.y = setpoints_ned_coords[i-1][1]
        # Set Z height
        tmp[i].pose.position.z = local_setpoints[0].pose.position.z
        # Set orientation
    return tmp

def local_to_ned_test():
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rospy.init_node('local_to_ned_test', anonymous=True)
    #Subscribe to get current position
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_data_callback)
    rospy.sleep(1)
    r = rospy.Rate(5)
    setpoints = [PoseStamped()] * 6

    #Set Takeoff Point
    setpoints[0] = current_pose 
    setpoints[0].header = Header()
    setpoints[0].header.stamp = rospy.Time.now()
    setpoints[0].pose.position.z += 1

    # Transform all setpoints to drone coordinates
    setpoints = define_drone_setpoints(setpoints)
    print(setpoints)
#    #ARM
#    arm_service(True)
#    print("REQUEST ARM")
#
#    x = threading.Thread(target=publisher_thread, args=(setpoints, setpoint_pub, SETPOINT_FREQ, FLY_TIME))
#    x.start()
#    #Set Offboard Mode and fly a square
#    mode_resp = mode_service(0,"OFFBOARD")
#    print("SWITCH TO OFFBOARD")
#    print("TAKEOFF")
#    x.join()
#
#    #Land
#    mode_service(0, "AUTO.LAND")
#    print("LANDING")

if __name__ == "__main__":
    # Parse Config
    config = configparser.ConfigParser()
    config.read('../config.ini')

    MAP_TO_DRONE_RAD = float(config['hardware_setup']['local_ned_offset_rad'])
    
    # Perform Test
    local_to_ned_test()
