import numpy as np

reading = input("What's the angle displayed when the drone points from anchor 12 to 10?\n")
reading = float(reading)
offset_rad = (360-reading)*np.pi/180
offset_rad_string = str(offset_rad)

with open('config.ini', 'r') as configfile:
    lines = configfile.readlines()

with open('config.ini', 'w') as configfile:
    for line in lines:
        if line.startswith("local_ned_offset_rad"):
            line = "local_ned_offset_rad = "+offset_rad_string
            print("offset is: "+str(360-reading)+" degrees") 
            print("local_ned_offset_rad set to "+offset_rad_string+" radians") 
        configfile.write(line)
