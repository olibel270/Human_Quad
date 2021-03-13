"path_functions.py"
"All functions here return sets of waypoints as numpy arrays of size nx4, where n is the number of waypoints for a manoeuver,"
"and each waypoint is a numpy array of the form [x,y,z,yaw(deg)]"
"Waypoints are defined in the local coordinate frame (xyz)"
import numpy as np
from transform_functions import yaw_rad_from_quaternion

def circle_waypoints(center, radius, number_of_turns = 1, waypoints_per_turn=8, start_angle=0):     
# center: 1x4 np.array, radius: radius of the circle (m), start_angle (rad)
    
    # initialize a numpy array of the right size for the waypoints
    num_waypoints = number_of_turns * waypoints_per_turn + 1
    waypoints_array = np.zeros([num_waypoints, 4])
    
    # for loop over rows of array:
    current_angle = start_angle
    for waypoint in waypoints_array:
        # Not most efficient, could be done with matrix mult, but for now:
        # yaw (degrees) points towards the center of the circle.
        waypoint[0] = radius*np.cos(current_angle)+center[0]
        waypoint[1] = radius*np.sin(current_angle)+center[1]
        waypoint[2] = center[2]
        waypoint[3] = current_angle * 180/np.pi + 180
        if(waypoint[3]>359): 
            waypoint[3] = waypoint[3]-360
        current_angle += 2*np.pi/waypoints_per_turn

    return waypoints_array

def helix_waypoints(center, radius, start_height, end_height, number_of_turns = 1, waypoints_per_turn=8, start_angle=0):     
# center: 1x4 np.array, radius: radius of the circle (m), start_angle (rad)
    
    # initialize a numpy array of the right size for the waypoints
    num_waypoints = number_of_turns * waypoints_per_turn + 1
    waypoints_array = np.zeros([num_waypoints, 4])
    
    # for loop over rows of array:
    current_angle = 0+start_angle
    height_increment = (end_height-start_height)/(number_of_turns*waypoints_per_turn)
    current_height = start_height
    for waypoint in waypoints_array:
        # Not most efficient, could be done with matrix mult, but for now:
        # yaw (degrees) points towards the center of the circle.
        waypoint[0] = radius*np.cos(current_angle)+center[0]
        waypoint[1] = radius*np.sin(current_angle)+center[1]
        waypoint[2] = current_height
        waypoint[3] = current_angle * 180/np.pi + 180
        if(waypoint[3]>359): 
            waypoint[3] = waypoint[3]-360
        current_angle += 2*np.pi/waypoints_per_turn
        current_height += height_increment

    return waypoints_array

def setpoint_reached(setpoint, current_pose, accuracy_pose=0.2, accuracy_yaw=5):
    x = current_pose.pose.position.x
    y = current_pose.pose.position.y
    z = current_pose.pose.position.z

    goal_x = setpoint.pose.position.x
    goal_y = setpoint.pose.position.y
    goal_z = setpoint.pose.position.z

    goal_yaw = yaw_rad_from_quaternion(setpoint.pose.orientation)
    yaw = yaw_rad_from_quaternion(current_pose.pose.orientation)
    distance_yaw = abs(goal_yaw-yaw)
    distance = np.sqrt((goal_x-x)**2 + (goal_y-y)**2 + (goal_z-z)**2)
    if(distance<=accuracy_pose and distance_yaw<=accuracy_yaw):
        return True
    else:
        return False
