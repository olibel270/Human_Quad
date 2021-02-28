import math

positions = [[10, 2, 23], [14, 5, 16], [64, 25, 16]]  # set points in the local xyz

alpha = 20  # angle between both x axis

OxyzNED = [1, 1, 1]  # position of origin Oxyz in NED coordinates

for x in positions:
    # fist paragraph, we apply the transformation as if both coordinates have the same origin
    a = x[0] * math.cos(alpha) + x[1] * math.sin(alpha)
    x[1] = -x[0] * math.sin(alpha) + x[1] * math.cos(alpha)
    x[0] = a
    x[2] = -x[2]

    # we add the difference in case both coordinates have different origins
    x[0] += OxyzNED[0]
    x[1] += OxyzNED[1]
    x[2] += OxyzNED[2]

print(positions)
