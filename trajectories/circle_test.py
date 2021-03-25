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
from path_functions import circle_waypoints, setpoint_reached

current_pose = PoseStamped()
MAP_TO_DRONE_RAD = 0
FLY_TIME = 22 
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
    i=0
    done=False
    while not done:
        if(setpoint_reached(setpoints[i], current_pose, accuracy_pose=1.2)):
            if(i==0):
                for j in range(40):
                    publisher.publish(setpoints[i])
                    r.sleep()
            if(i==97):
                for j in range(15):
                    publisher.publish(setpoints[i])
                    r.sleep()     
            if(i==98):
                for j in range(25):
                    publisher.publish(setpoints[i])
                    r.sleep()
            i+=1
            if(i>=len(setpoints)):
                done = True
                continue
            print("To setpoint: " + str(i))
            setpoint = setpoints[i]
        publisher.publish(setpoint)
        r.sleep()
    return

def define_drone_setpoints(starting_setpoint):
    # Define local setpoint coordinates
    new_setpoints_coords = circle_waypoints(np.array([0,-1,1,180]),1,number_of_turns=3, waypoints_per_turn=32)
    new_setpoints_coords = np.append(new_setpoints_coords, [[3.5,-1,1,180]],axis=0)
    print(new_setpoints_coords)
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
    mode_service(0, "AUTO.LAND")
    print("LANDING")

if __name__ == "__main__":
    # Perform Test
    local_to_ned_test()
