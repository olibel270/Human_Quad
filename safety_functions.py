#!/usr/bin/env python

"""Contains functions to call for safety features.
   -Battery level for preflight check
   -preflight_positioning check"""

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from trajectories.transform_functions import ned_to_xyz

def get_battery_level():
    rospy.init_node('battery_listener')
    msg = rospy.wait_for_message('/mavros/battery', BatteryState)
    return msg.percentage

def act_launch_is_safe_battery():
    if get_battery_level()>0.7:
        return True
    else:
        print("Battery too low to launch act. Please charge or replace battery")
        return False

def get_position_xyz():
    rospy.init_node('position_listener')
    msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
    n = msg.pose.position.x
    e = msg.pose.position.y
    d = msg.pose.position.z
    (x,y,z) = ned_to_xyz(n,e,d)
    return (x,y,z)

def act_launch_is_safe_position():
    (x,y,z) = get_position_xyz()
    if (x == 0 and y ==0):
        print("Position not being received. Cannot Launch")
        return False
    (x_min, x_max) = (-1, 1)
    (y_min, y_max) = (-2,-0.5)
    x_ok = True if (x<x_max and x>x_min) else False
    y_ok = True if (y<y_max and y>y_min) else False
    print(x,y,z)
    print("x_ok: " + str(x_ok))
    print("y_ok: " + str(y_ok))
    return True if (x_ok and y_ok) else False

print(act_launch_is_safe_position())
