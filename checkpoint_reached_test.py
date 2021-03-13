#!/usr/bin/env python
"""ROS node that tests offboard mode"""

import rospy
import sys, threading
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Header
from trajectories.path_functions import setpoint_reached

current_pose = None
FLY_TIME = 5 #20s total fly time for the test. Should be setpoint distance based, but good for now + it's just a test
SETPOINT_FREQ = 5

def pose_data_callback(pose):
    global current_pose
    current_pose = pose

def publisher_thread(setpoint, publisher, rate, fly_time=0):
    r = rospy.Rate(rate)
    if fly_time != 0:
        iterations = fly_time / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    else:
        iterations = 5 / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    count = 0
    print("Iterations: ", iterations)
    setpoint_reached(setpoint,current_pose)
    while count<iterations:
        publisher.publish(setpoint)
        if(count==iterations*(1/5)):#4s
            setpoint.pose.position.x -= 1
            setpoint_reached(setpoint,current_pose)
        if(count==iterations*(2/5)):#8s
            setpoint.pose.position.y -= 1
            setpoint_reached(setpoint,current_pose)
        if(count==iterations*(3/5)):#12s
            setpoint.pose.position.x += 1
            setpoint_reached(setpoint,current_pose)
        if(count==iterations*(4/5)):#16s
            setpoint.pose.position.y += 1
            setpoint.pose.position.z -= 1
            setpoint.pose.orientation = Quaternion(0,0,0.707,0.707)
            setpoint_reached(setpoint,current_pose,accuracy_pose=0.3)
        r.sleep()
        count += 1


def offboard_test():
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rospy.init_node('offboard_test', anonymous=True)
    #Subscribe to get current position
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_data_callback)
    rospy.sleep(1)
    r = rospy.Rate(5)

    #ARM
    arm_service(True)

    #Set Takeoff Point
    setpoint = current_pose 
    setpoint.header = Header()
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.z += 1
    rospy.sleep(0.1) #for some reason otherwise the setpoint doesn't update fast enough
    
    print(setpoint)
    print(current_pose)
    x = threading.Thread(target=publisher_thread, args=(setpoint, setpoint_pub, SETPOINT_FREQ, FLY_TIME))
    x.start()
    #Set Offboard Mode and fly a square
    mode_resp = mode_service(0,"OFFBOARD")
    x.join()

    #Land
    mode_service(0, "AUTO.LAND")

if __name__ == "__main__":
    offboard_test()
