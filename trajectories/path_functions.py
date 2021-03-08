"path_functions.py"
"All functions here return sets of waypoints as numpy arrays of size nx4, where n is the number of waypoints for a manoeuver,"
"and each waypoint is a numpy array of the form [x,y,z,yaw(deg)]"
"Waypoints are defined in the local coordinate frame (xyz)"
import numpy as np

def circle_waypoints(center, radius, number_of_turns = 1, waypoints_per_turn=8, start_angle=0):     
# center: 1x4 np.array, radius: radius of the circle (m), start_angle (rad)
    
    # initialize a numpy array of the right size for the waypoints
    num_waypoints = number_of_turns * waypoints_per_turn + 1
    waypoints_array = np.zeros([num_waypoints, 4])
    
    # for loop over rows of array:
    current_angle = 0+start_angle
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
