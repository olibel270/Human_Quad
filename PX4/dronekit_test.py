from dronekit import connect

# Connect to UDP endpoint.
vehicle = connect('/dev/ttyACM0', wait_ready=True)
# Use returned Vehicle object to query device state - e.g. to get the mode:
print("Mode: %s" % vehicle.mode.name)
