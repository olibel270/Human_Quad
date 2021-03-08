import numpy as np
from configparser import ConfigParser
from geometry_msgs.msg import PoseStamped, Quaternion

def get_map_to_drone_rad():
    config = ConfigParser()
    config.read('/home/ubuntu/Human_Quad/config.ini')
    return float(config['hardware_setup']['local_ned_offset_rad'])

# Inputs: nx4 array of points in the local frame, rotation radian
# Outs: rotated nx4 array
def local_to_ned(points):
    angle_rad = get_map_to_drone_rad()
    tmp = np.zeros([len(points), 4])
    for i, point in enumerate(points):
        n = point[0] * np.cos(angle_rad) - point[1] * np.sin(angle_rad)
        e = -(point[0] * np.sin(angle_rad) + point[1] * np.cos(angle_rad))
        d = -point[2]
        yaw = point[3]*np.pi/180 + get_map_to_drone_rad()
        tmp[i] = np.array([n,e,d,yaw])
    return tmp

def quaternion_from_rad_yaw(yaw_in_rads):
    quaternion = Quaternion()
    quaternion.w = np.cos(yaw_in_rads/2)
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = np.sin(yaw_in_rads/2)
    return quaternion

# Input: nx4 numpy array [x,y,z,yaw(degrees)]
# Output: nx1 array of geomtry_msgs.msg.PoseStamped
def array_to_setpoints(array):
    ned_array = local_to_ned(array)
    setpoints_array = []
    for i in range(len(ned_array)):
        setpoints_array.append(PoseStamped())
        setpoints_array[i].pose.position.y = ned_array[i][0]
        setpoints_array[i].pose.position.x = ned_array[i][1]
        setpoints_array[i].pose.position.z = ned_array[i][2]
        setpoints_array[i].pose.orientation = quaternion_from_rad_yaw(ned_array[i][3])
    return setpoints_array
