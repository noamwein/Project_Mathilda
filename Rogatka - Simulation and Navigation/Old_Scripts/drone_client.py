from interfaces import DroneClient

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import time
from typing import Tuple
import math
import logging

import enum


class State(enum.Enum):
    TAKEOFF = 0
    PATROLE = 1
    ROTATION = 2
    MOVEMENT = 3
    ON_TARGET = 4
    LANDING = 5



class BasicClient(DroneClient):
    def __init__(self, connection_string: str, initial_altitude: float, max_altitude: float, min_battery_percent: float, logger: logging.Logger):
        self.connection_string = connection_string
        self.initial_altitude = initial_altitude
        self.max_altitude = max_altitude
        self.min_battery_percent = min_battery_percent
        self.vehicle = None
        self.state = -1
        self.logger = logger
    
    def connect(self):
        print('Connecting...')
        self.vehicle = connect(self.connection_string, wait_ready=True)
        print('Connected!')
    
    def takeoff(self):
        print("Taking off...")
        self.state = State.TAKEOFF
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(self.initial_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= 0.95*self.initial_altitude:
                break
            time.sleep(1)
        
        print("In the air!!")
    
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
        print(f"max turn rate: {max_yaw_rate}")
        yaw_rate = max_yaw_rate * speed_factor

        current_heading = self.vehicle.heading

        new_heading = (current_heading + angle) % 360


        # MAVLink command to rotate the drone
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,               # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,                  # confirmation
            new_heading,          # param 1: Yaw angle
            yaw_rate,           # param 2: Yaw rate
            0,                  # param 3: Absolute heading
            0, 0, 0, 0          # param 4-7: Not used
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
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)     # yaw, yaw_rate (not used)
        self.vehicle.send_mavlink(msg)
        self.vehicle.commands.upload()
    
    def goto_target(self, target_position):
        if self.state == State.TAKEOFF: #acctually finished takeoff
            print("Starting rotation")
            self.face_target(target_position)
            self.state = State.ROTATION
        elif self.state == State.ROTATION: #do nothing if not aligned, else start moving
            if is_aligned(target_position):
                print("Aligned!")
                self.set_speed(1, 0, 0)
                self.state = State.MOVEMENT
        elif self.state == State.MOVEMENT: # if not aligned than rotate again, else fix speed if needed, othwise do nothing
            if not is_aligned(target_position):
                print("Fixing alignement...")
                self.set_speed(0, 0, 0)
                self.face_target(target_position)
                self.state = State.ROTATION
            else:
                current_speed = self.vehicle.groundspeed
                if abs(current_speed - 1) > 0.1:
                    print(f"Speed is {current_speed}, fixing!")
                    self.set_speed(1, 0, 0)
                    time.sleep(0.1)
        elif self.state == State.ON_TARGET: #oops... means we are not on target and need to fix - start the process over
            print("Starting position fix")
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
            print("Rotating", direction, "degrees...")
            self.rotate(direction)

        
    def return_to_launch(self):
        self.vehicle.mode = VehicleMode("RTL")

    def disconnect(self):
        self.vehicle.close()

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