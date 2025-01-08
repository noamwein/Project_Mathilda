import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from pymavlink import mavutil

import time
import math
import logging

from typing import Tuple
import enum

from BirdBrain.interfaces import DroneClient

class State(enum.Enum):
    """
    Enum representing the states of the drone.
    """
    TAKEOFF = 0
    PATROL = 1
    ROTATION = 2
    MOVEMENT = 3
    ON_TARGET = 4
    LANDING = 5

class BasicClient(DroneClient):
    """
    A basic client for controlling a drone using pymavlink.

    Attributes:
        connection_string (str): MAVLink connection string.
        initial_altitude (float): Initial takeoff altitude in meters.
        max_altitude (float): Maximum allowable altitude in meters.
        min_battery_percent (float): Minimum battery percentage before landing.
        mavlink_connection: MAVLink connection object.
        state (State): Current state of the drone.
        logger (logging.Logger): Logger for logging drone activity.
    """
    def __init__(self, connection_string: str, initial_altitude: float, max_altitude: float, min_battery_percent: float, logger: logging.Logger):
        """
        Initialize the BasicClient object.

        Args:
            connection_string (str): MAVLink connection string.
            initial_altitude (float): Desired altitude for takeoff.
            max_altitude (float): Maximum allowable altitude.
            min_battery_percent (float): Minimum battery percentage for safe operation.
            logger (logging.Logger): Logger instance.
        """
        self.connection_string = connection_string
        self.initial_altitude = initial_altitude
        self.max_altitude = max_altitude
        self.min_battery_percent = min_battery_percent
        self.mavlink_connection = None
        self.state = -1
        self.logger = logger

    def connect(self):
        """
        Connect to the drone using the MAVLink connection string.
        """
        print('Connecting...')
        self.mavlink_connection = mavutil.mavlink_connection(self.connection_string)
        self.mavlink_connection.wait_heartbeat()  # Wait for a heartbeat to ensure connection
        print('Connected!')

    def takeoff(self):
        """
        Arm the drone and perform a takeoff to the specified initial altitude.
        """
        print("Taking off...")
        self.state = State.TAKEOFF

        # Set mode to GUIDED
        self.mavlink_connection.set_mode_apm('GUIDED')

        # Send command to arm the drone
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Confirm the drone is armed
        ack = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True)
        if ack.result != 0:
            raise RuntimeError("Failed to arm vehicle")

        # Command takeoff
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            self.initial_altitude
        )

        # Wait until the drone reaches the target altitude
        while True:
            alt = self.mavlink_connection.messages['GPS_RAW_INT'].alt
            print(alt)
            if alt >= 0.95 * self.initial_altitude:  # Check if altitude is close to the target
                break
            time.sleep(0.1)

        print("In the air!!")

    def rotate(self, angle, speed_factor=0.8):
        """
        Rotate the drone to a specified yaw angle.

        Args:
            angle (float): Desired yaw angle in degrees.
            speed_factor (float): Fraction of the maximum yaw speed (default is 0.8).
        """
        max_yaw_rate = 200  # Default max yaw rate in deg/s
        yaw_rate = max_yaw_rate * speed_factor

        # MAVLink command to rotate the drone
        msg = self.mavlink_connection.mav.command_long_encode(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle, yaw_rate, 0, 1, 0, 0, 0
        )
        self.mavlink_connection.mav.send(msg)

    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Set the drone's velocity in the local NED frame.

        Args:
            velocity_x (float): Velocity in the north direction (m/s).
            velocity_y (float): Velocity in the east direction (m/s).
            velocity_z (float): Velocity in the downward direction (m/s).
        """
        # Send MAVLink command to set velocity
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference
            0b0000111111000111,  # Type mask (enable only velocity)
            0, 0, 0,  # Position (not used)
            velocity_x, velocity_y, velocity_z,  # Velocity components
            0, 0, 0,  # Acceleration (not used)
            0, 0  # Yaw, yaw rate (not used)
        )

    def goto_target(self, target_position):
        """
        Navigate the drone to a target position.

        Args:
            target_position (Tuple[int, int]): Target position as (x, y) coordinates.
        """
        if self.state == State.TAKEOFF:  # Acctually finished takeoff, start aligning to the target
            print("Starting rotation")
            self.face_target(target_position)
            self.state = State.ROTATION
        elif self.state == State.ROTATION:  # If aligned, then start moving
            if self.is_aligned(target_position):
                print("Aligned!")
                self.set_velocity(1, 0, 0)  # Move forward
                self.state = State.MOVEMENT
        elif self.state == State.MOVEMENT:  # Re-align or fix speed if necessary
            if not self.is_aligned(target_position):
                print("Fixing alignment...")
                self.set_velocity(0, 0, 0)  # Stop
                self.face_target(target_position)
                self.state = State.ROTATION
            else:
                current_speed = self.get_groundspeed()
                if abs(current_speed - 1) > 0.05:
                    print(f"Speed is {current_speed}, fixing!")
                    self.set_velocity(1, 0, 0)
                    time.sleep(0.1)
        elif self.state == State.ON_TARGET: # Oops... means we are not on target and need to fix - start the process over
            print("Starting position fix")
            self.face_target(target_position)
            self.state = State.ROTATION
    
    def has_stopped(self, error_tolerence=0.05):
        return abs(self.get_groundspeed()) < error_tolerence
    
    def stop_movement(self):
        self.set_velocity(0, 0, 0)
        self.state = State.ON_TARGET
    
    def is_on_target(self, target_position: Tuple[int, int], error_tolerence=0.1):
        return math.sqrt(target_position[0] ** 2 + target_position[1] ** 2) < error_tolerence

    def is_aligned(self, target_position, error_tolerance=5):
        """
        Check if the drone is aligned with the target position.

        Args:
            target_position (Tuple[int, int]): Target position as (x, y) coordinates.
            error_tolerance (float): Allowed error tolerance in degrees (default is 5).

        Returns:
            bool: True if aligned, False otherwise.
        """
        return abs(target_position[0]) < error_tolerance and target_position[1] >= 0

    def face_target(self, target_position):
        """
        Rotate the drone to face the target position.

        Args:
            target_position (Tuple[int, int]): Target position as (x, y) coordinates.
        """
        direction = self.calculate_direction(target_position)  # Calculate direction to face
        print("Rotating", direction, "degrees...")
        self.rotate(direction)

    def return_to_launch(self):
        """
        Command the drone to return to the launch location.
        """
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def disconnect(self):
        """
        Disconnect the MAVLink connection.
        """
        if self.mavlink_connection:
            self.mavlink_connection.close()

    def calculate_direction(self, target_position):
        """
        Calculate the yaw angle needed to face the target position.

        Args:
            target_position (Tuple[int, int]): Target position as (x, y) coordinates.

        Returns:
            float: Calculated yaw angle in degrees.
        """
        x, y = target_position

        # Calculate angle in radians
        angle_radians = math.atan2(y, x)

        # Adjust to align with the positive y-axis
        yaw_angle = (math.pi / 2) - angle_radians

        # Normalize to [-π, π]
        yaw_angle = (yaw_angle + math.pi) % (2 * math.pi) - math.pi

        # Convert to degrees
        return (yaw_angle * 360) / (2 * math.pi)

    def get_groundspeed(self):
        """
        Get the current groundspeed of the drone.

        Returns:
            float: Current groundspeed in m/s.
        """
        msg = self.mavlink_connection.recv_match(type='VFR_HUD', blocking=True)
        return msg.groundspeed
