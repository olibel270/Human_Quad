#!/usr/bin/env python

"""Contains functions to call for safety features.
   -Battery level for preflight check
   -preflight_positioning check"""

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from ..trajectories.transform_functions import ned_to_xyz

def get_battery_level():
    rospy.init_node('battery_listener')
    msg = rospy.wait_for_message('/mavros/battery', BatteryState)
    return msg.percentage

def act_launch_is_safe_battery():
    return True if get_battery_level()>0.7 else False

def get_position_xyz():
    rospy.init_node('position_listener')
    msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    (x,y,z) = ned_to_xyz(n,e,d)
    return (x,y,z)

def act_launch_is_safe_position():
    (x,y,z) = get_position_xyz()
    print(x,y,z)
    if (x == 0 and y ==0):
        print("Position not being received. Cannot Launch")
        return False
    (x_min, x_max) = (-1, 1)
    (y_min, y_max) = (-0.5,-2)
    x_ok = True if (x<x_max and x>x_min) else False
    y_ok = True if (y<y_max and y>y_min) else False
    return True if (x_ok and y_ok) else False

print(act_launch_is_safe_position())
