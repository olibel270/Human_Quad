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
from transform_functions import array_to_setpoints
from path_functions import circle_waypoints

current_pose = PoseStamped()
MAP_TO_DRONE_RAD = 0
FLY_TIME = 22 #60s total fly time for the test. Should be setpoint distance based, but good for now + it's just a test
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
        if(count==iterations*(6/22)):#6s
            setpoint = setpoints[1]
            print("Performing CIRCLE!")
            print("To Setpoint 1")
        if(count==iterations*(8/22)):#12s
            setpoint = setpoints[2] 
            print("To Setpoint 2")
        if(count==iterations*(10/22)):#18s
            setpoint = setpoints[3]
            print("To Setpoint 3")
        if(count==iterations*(12/22)):#24s
            setpoint = setpoints[4]
            print("To Setpoint 4")
        if(count==iterations*(14/22)):#30s
            setpoint = setpoints[5]
            print("To Setpoint 5")
        if(count==iterations*(16/22)):#36s
            setpoint = setpoints[6]
            print("To Setpoint 6")
        if(count==iterations*(18/22)):#42s
            setpoint = setpoints[7]
            print("To Setpoint 7")
        if(count==iterations*(20/22)):#48s
            setpoint = setpoints[8]
            print("To Setpoint 8")
        if(count==iterations*(22/22)):#54s
            setpoint = setpoints[9]
            print("To Setpoint 9")
        r.sleep()
        count += 1

def define_drone_setpoints(starting_setpoint):
    # Define local setpoint coordinates
    new_setpoints_coords = circle_waypoints(np.array([0,-1,1.5,1,180]),1)
    new_setpoints = array_to_setpoints(new_setpoints_coords)
    tmp = [PoseStamped()] * (1+len(new_setpoints_coords))
    for i,setpoint in enumerate(tmp):
        if(i==0):
            tmp[i] = starting_setpoint
        else:
            tmp[i] = new_setpoints[i-1]
            tmp[i].pose.position.z = tmp[0].pose.position.z
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
    setpoint_start = PoseStamped()

    #Set Takeoff Point
    setpoint_start = current_pose 
    setpoint_start.header = Header()
    setpoint_start.header.stamp = rospy.Time.now()
    setpoint_start.pose.position.z += 1

    # Transform all setpoints to drone coordinates
    setpoints = define_drone_setpoints(setpoint_start)
    print(setpoints)
    #ARM
    arm_service(True)
    print("REQUEST ARM")

    x = threading.Thread(target=publisher_thread, args=(setpoints, setpoint_pub, SETPOINT_FREQ, FLY_TIME))
    x.start()
    #Set Offboard Mode and fly a square
    mode_resp = mode_service(0,"OFFBOARD")
    print("SWITCH TO OFFBOARD")
    print("TAKEOFF")
    x.join()

    #Land
    #mode_service(0, "AUTO.LAND")
    #print("LANDING")

if __name__ == "__main__":
    # Perform Test
    local_to_ned_test()
