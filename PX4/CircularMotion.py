import math
import numpy as np
from scipy.spatial.transform import Rotation as R

arr = []  # empty collection of points.


def circular_waypoints(x, z, r,
                       n):  # x is initial x distace between the drone and the circle, z: height of circle, r: radius, n: number of rotations
    p = np.array([x, math.sqrt(r ** 2 - x ** 2), z])  # initial position of the drone (test)

    v = R.from_quat([0, 0, np.sin(np.pi / n), np.cos(np.pi / n)])  # unit rotation quaternion

    i = 1
    while i < n + 1:  # apply the rotation on the points n times
        l = v.apply(p)

        p = l

        arr.append(l)
        i += 1
    print(arr)  # print all the waypoints


print("List of Waypoints from the circle with x = 1, z = 10, r = 1 and n = 4 :")
circular_waypoints(1, 10, 1, 4)

print("A specific Waypoint from collection :")
print(arr[2])  # print only a specific waypoint


def pointcc(x, y, xx, yy):  # This function makes the quadcopter's Yawn angle points toward the center
    # of the circular path, x and y is pos. of drone, xx and yy are the position of the center of the circle
    dx = x - xx
    dy = y - yy

    if dx < 0:
        return math.pi + math.atan(dy / dx)
        return math.atan(dy / dx)
    elif dy > 0:
        return math.atan(dy / dx)
    else:
        return 2 * math.pi + math.atan(dy / dx)


print("Yawn angle in radian for the quad at (-4, -7) to face the center of circle at (2, 3) :")
YawAng = pointcc(-4, -7, 2, 3)  # test

# v = R.from_quat([0, 0, np.sin(np.pi / n), np.cos(np.pi / n)])


r = R.from_euler('z', YawAng, degrees=True)

rquat = r.as_quat()

print("The Yawn angle as a quaternion")
print(rquat)

