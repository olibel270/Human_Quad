#!/usr/bin/env python
"""ROS node that tests offboard mode"""

import rospy
import sys, threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Header

current_pose = None
FLY_TIME = 20 #20s total fly time for the test. Should be setpoint distance based, but good for now + it's just a test
SETPOINT_FREQ = 5

def pose_data_callback(pose):
    global current_pose
    current_pose = pose

def publisher_thread(obj, publisher, rate, fly_time=0):
    r = rospy.Rate(rate)
    if fly_time != 0:
        iterations = fly_time / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    else:
        iterations = 5 / (r.sleep_dur.secs + r.sleep_dur.nsecs/1e9)
    count = 0
    print("Iterations: ", iterations)
    while count<iterations:
        publisher.publish(obj)
        if(count==iterations/(1/5)):#8s
            setpoint.pose.position.x -= 1.5
        if(count==iterations/(2/5)):#12s
            setpoint.pose.position.y -= 1.5
        if(count==iterations/(3/5):#16s
            setpoint.pose.position.x += 1.5
        if(count==iterations/(4/5):#20s
            setpoint.pose.position.y += 1.5
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
    
    print(setpoint)
    x = threading.Thread(target=publisher_thread, args=(setpoint, setpoint_pub, SETPOINT_FREQ, FLY_TIME))
    x.start()
    #Set Offboard Mode and fly a square
    mode_resp = mode_service(0,"OFFBOARD")
    x.join()

    #Land
    mode_service(0, "AUTO.LAND")

if __name__ == "__main__":
    offboard_test()
