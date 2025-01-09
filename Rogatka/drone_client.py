import collections.abc

from BirdBrain.interfaces import DroneClient

collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import time
from datetime import datetime

from typing import Tuple
import math
import logging

import enum

import os

MAXIMUM_DISTANCE = 12

class State(enum.Enum):
    TAKEOFF = 0
    PATROLE = 1
    ROTATION = 2
    MOVEMENT = 3
    ON_TARGET = 4
    LANDING = 5


class BasicClient(DroneClient):
    def __init__(self, connection_string: str, initial_altitude: float, max_altitude: float, min_battery_percent: float,
                 logger: logging.Logger):
        self.connection_string = connection_string
        self.initial_altitude = initial_altitude
        self.max_altitude = max_altitude
        self.min_battery_percent = min_battery_percent
        self.vehicle = None
        self.state = -1
        self.logger = logger
        self.terminated = False

        # Set up logging to file
        log_folder = "../Flight Logs"
        os.makedirs(log_folder, exist_ok=True)
        start_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = os.path.join(log_folder, f"flight-{start_time}.log")

        logging.basicConfig(
            filename=log_filename,
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%H:%M:%S'
        )
    
    def log_and_print(self, message: str):
        print(message)
        logging.info(message)

    def connect(self):
        self.log_and_print('Connecting...')
        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.log_and_print('Connected!')

    def takeoff(self):
        self.log_and_print("Taking off...")
        self.state = State.TAKEOFF
        self.vehicle.mode = VehicleMode("GUIDED")
        # Download the vehicle waypoints (commands). Wait until download is complete.
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        self.vehicle.armed = True

        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.wait_simple_takeoff(self.initial_altitude)

        self.log_and_print("In the air!!")
    
    def move_forward(self, distance):
        """
        Moves the drone forward in the direction of its current heading.
        """
        self.log_and_print(f"Moving forward {distance} meters")

        current_location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading  # Current heading in degrees (0-360)

        # Calculate the target location based on heading
        target_location = calculate_target_location(current_location, heading, distance)
        self.vehicle.simple_goto(target_location)

        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance_to_target = get_distance_meters(current_location, target_location)

            if distance_to_target <= 0.5:  # Consider 1 meter as the acceptable threshold
                self.log_and_print("Reached target location")
                break

            time.sleep(1)

    def rotate(self, angle, speed_factor=0.8):
        """
        Rotate the drone by a fixed yaw angle at a specified speed factor.

        :param vehicle: Vehicle object from DroneKit
        :param yaw_angle: Desired yaw angle change in degrees
        :param speed_factor: Fraction of max yaw speed (0.0 to 1.0)
        :param relative: If True, yaw_angle is relative to current heading
        """
        # Get the maximum yaw rate from the drone's parameters (default 200°/s if unknown)
        max_yaw_rate = self.vehicle.parameters.get('ATC_RATE_Y_MAX', 20000) / 100.0  # Convert from centidegrees/sec
        self.log_and_print(f"max turn rate: {max_yaw_rate}")
        yaw_rate = max_yaw_rate * speed_factor

        current_heading = self.vehicle.heading

        new_heading = (current_heading + angle) % 360

        # MAVLink command to rotate the drone
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            new_heading,  # param 1: Yaw angle
            yaw_rate,  # param 2: Yaw rate
            0,  # param 3: Absolute heading
            0, 0, 0, 0  # param 4-7: Not used
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def set_speed(self, velocity_x: float, velocity_y: float, velocity_z: float):
        """
        Move vehicle in direction based on specified velocity vectors.

        Parameters:
            vehicle (Vehicle): The connected DroneKit vehicle object.
            velocity_x (float): Velocity in m/s in the north direction.
            velocity_y (float): Velocity in m/s in the east direction.
            velocity_z (float): Velocity in m/s in the downward direction.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)
        self.vehicle.send_mavlink(msg)
        self.vehicle.commands.upload()

    def goto_target(self, target_position):
        if self.state == State.TAKEOFF:  # acctually finished takeoff
            self.log_and_print("Starting rotation")
            self.face_target(target_position)
            self.state = State.ROTATION
        elif self.state == State.ROTATION:  # do nothing if not aligned, else start moving
            if is_aligned(target_position):
                self.log_and_print("Aligned!")
                self.set_speed(1, 0, 0)
                self.state = State.MOVEMENT
        elif self.state == State.MOVEMENT:  # if not aligned than rotate again, else fix speed if needed, othwise do nothing
            if self.distance_from_home() > MAXIMUM_DISTANCE:
                self.log_and_print("Stafty termination because the drone went too far...")
                self.set_speed(0, 0, 0)
                self.state = State.LANDING
                self.terminated = True
            elif not is_aligned(target_position):
                self.log_and_print("Fixing alignement...")
                self.set_speed(0, 0, 0)
                self.face_target(target_position)
                self.state = State.ROTATION
            else:
                current_speed = self.vehicle.groundspeed
                if abs(current_speed - 1) > 0.05:
                    self.log_and_print(f"Speed is {current_speed}, fixing!")
                    self.set_speed(1, 0, 0)
                    time.sleep(0.1)
        elif self.state == State.ON_TARGET:  # oops... means we are not on target and need to fix - start the process over
            self.log_and_print("Starting position fix")
            self.face_target(target_position)
            self.state = State.ROTATION

    def has_stopped(self, error_tolerence=0.05):
        return abs(self.vehicle.groundspeed) < error_tolerence

    def stop_movement(self):
        self.set_speed(0, 0, 0)
        self.state = State.ON_TARGET

    def is_on_target(self, target_position: Tuple[int, int], error_tolerence=0.1):
        return math.sqrt(target_position[0] ** 2 + target_position[1] ** 2) < error_tolerence

    def face_target(self, target_position):
        direction = calculate_direction(target_position)
        self.log_and_print(f"Rotating {direction} degrees...")
        self.rotate(direction)

    def return_to_launch(self):
        self.log_and_print("Returning home!")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(1)

    def distance_from_home(self):
        # Get the home location
        home_location = self.vehicle.home_location
        current_location = self.vehicle.location.global_relative_frame
        dist = get_distance_meters(home_location, current_location)
        return dist

    def disconnect(self):
        self.log_and_print("Disconnecting.")
        self.vehicle.close()
    
    def mission_terminated(self):
        return self.terminated


def calculate_direction(target_position: Tuple[int, int]):
    """
    Calculate the yaw angle needed to align a pixel to the positive y-axis.

    :param pixel: Tuple (x, y) where (0, 0) is the screen center
    :return: Yaw angle in degrees (relative to the current heading)
    """
    x, y = target_position

    # Calculate the angle in radians
    angle_radians = math.atan2(y, x)

    # Adjust to align with the positive y-axis (π/2 radians)
    yaw_angle = (math.pi / 2) - angle_radians

    # Normalize to the range [-π, π]
    yaw_angle = (yaw_angle + math.pi) % (2 * math.pi) - math.pi

    return (yaw_angle * 360) / (2 * math.pi)

def is_aligned(target_position: Tuple[int, int], error_tolerence=5) -> bool:
    return (abs(target_position[0]) < error_tolerence) and (target_position[1] >= 0)

def get_distance_meters(location1, location2):
    """
    Returns the ground distance in meters between two LocationGlobalRelative objects.
    """
    lat_diff = location2.lat - location1.lat
    lon_diff = location2.lon - location1.lon
    return math.sqrt((lat_diff * 111320.0) ** 2 + (lon_diff * 111320.0) ** 2)

def calculate_target_location(current_location, heading, distance):
    """
    Calculates the target location based on the current location, heading, and distance.
    """
    earth_radius = 6378137.0  # Earth's radius in meters

    # Convert heading and distance to radians
    heading_rad = math.radians(heading)
    distance_rad = distance / earth_radius

    # Current latitude and longitude in radians
    lat1 = math.radians(current_location.lat)
    lon1 = math.radians(current_location.lon)

    # Calculate target latitude and longitude
    lat2 = math.asin(math.sin(lat1) * math.cos(distance_rad) +
                     math.cos(lat1) * math.sin(distance_rad) * math.cos(heading_rad))
    lon2 = lon1 + math.atan2(math.sin(heading_rad) * math.sin(distance_rad) * math.cos(lat1),
                             math.cos(distance_rad) - math.sin(lat1) * math.sin(lat2))

    # Convert back to degrees
    target_lat = math.degrees(lat2)
    target_lon = math.degrees(lon2)

    return LocationGlobalRelative(target_lat, target_lon, current_location.alt)
