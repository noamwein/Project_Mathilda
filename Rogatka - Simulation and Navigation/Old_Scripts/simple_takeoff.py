import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import time

# Define the connection string
# Replace "127.0.0.1:5760" with the appropriate IP and port if different
connection_string = "tcp:127.0.0.1:5763"

# Connect to the Vehicle
print("Connecting to vehicle on: {}".format(connection_string))
vehicle = connect(connection_string, wait_ready=True)

# Print basic vehicle state
print("Vehicle Connected")
print("GPS: {}".format(vehicle.gps_0))
print("Battery: {}".format(vehicle.battery))
print("Mode: {}".format(vehicle.mode.name))    # Vehicle mode
print("Is Armable?: {}".format(vehicle.is_armable))
print("System status: {}".format(vehicle.system_status.state))


# Arm the drone
print("Arming vehicle...")
vehicle.mode = VehicleMode("GUIDED")  # Set to GUIDED mode
vehicle.armed = True

# Wait until the vehicle is armed
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Vehicle is armed")

# Takeoff to 3 meters
print("Taking off!")
vehicle.simple_takeoff(3)  # Take off to 3 meters

# Wait until the vehicle reaches the target altitude
while True:
    altitude = vehicle.location.global_relative_frame.alt
    print("Altitude: {:.1f} meters".format(altitude))
    if altitude >= 9.5:  # Target altitude - tolerance
        print("Reached target altitude")
        break
    time.sleep(1)

# Land the drone
print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Close the vehicle connection
vehicle.close()
print("Connection closed")
