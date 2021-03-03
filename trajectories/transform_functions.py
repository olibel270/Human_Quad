import numpy as np

# Inputs: nx4 array of points in the local frame, rotation radian
# Outs: rotated nx4 array
def local_to_ned(points, angle_rad):
    tmp = np.zeros([len(points), 4])
    for i, point in enumerate(points):
        n = point[0] * np.cos(angle_rad) + point[1] * np.sin(angle_rad)
        e = -(-point[0] * np.sin(angle_rad) + point[1] * np.cos(angle_rad))
        d = -point[2]
        tmp[i] = np.array([n,e,d,point[3]])
    return tmp

def quaternion_from_rad_yaw(yaw_in_rads):
    w = np.cos(yaw_in_rads/2)
    x = 0
    y = 0
    z = np.sin(yaw_in_rads/2)
    return w,x,y,z
