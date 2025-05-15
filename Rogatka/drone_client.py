import collections.abc

from BirdBrain.interfaces import DroneClient, require_guided, ImageDetection, Source, Waypoint, MovementAction

from .utils import get_distance_meters, calculate_target_location

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode, LocationGlobalRelative

from pymavlink import mavutil

import time
from datetime import datetime

from typing import Tuple, List
import math
import logging
from collections import deque

import enum

import os

import numpy as np

from BirdBrain.settings import (MAXIMUM_DISTANCE,
                                KILL_SWITCH_CHANNEL,
                                KILL_SWITCH_MODE,
                                YAW_TOLERANCE_THRESHOLD,
                                DROP_RADIUS,
                                MAX_SPEED,
                                YAW_TOLERANCE_RADIUS,
                                KP_V, KI_V, KD_V,
                                KP_YAW, KI_YAW, KD_YAW,
                                MISS_LIMIT,
                                YAW_INTEGRAL_MAX,
                                VEL_INTEGRAL_MAX, CENTERED_X, CENTERED_Y, PIXELS_PER_RAD, INITIAL_ALTITUDE)

class State(enum.Enum):
    TAKEOFF = 0
    PATROLE = 1
    ROTATION = 2
    MOVEMENT = 3
    ON_TARGET = 4
    LANDING = 5


class BasicClient(DroneClient):
    def __init__(self, connection_string: str, max_altitude: float, min_battery_percent: float,
                 logger: logging.Logger):
        self.connection_string = connection_string
        self.max_altitude = max_altitude
        self.min_battery_percent = min_battery_percent
        self.vehicle = None
        self.state = -1
        self.logger = logger
        self.done = False

        # PID state
        self._prev_time = None
        # Previous errors
        self._prev_error_vx = 0.0
        self._prev_error_vy = 0.0
        self._prev_error_yaw = 0.0
        # Integral histories (for sliding window)
        self._yaw_history = deque(maxlen=YAW_INTEGRAL_MAX)
        self._vx_history = deque(maxlen=VEL_INTEGRAL_MAX)
        self._vy_history = deque(maxlen=VEL_INTEGRAL_MAX)
        # Integral sums
        self._integral_yaw = 0.0
        self._integral_vx = 0.0
        self._integral_vy = 0.0
        # Miss counters
        self._yaw_miss = 0
        self._vel_miss = 0

        # Arm confirm
        self.arm_confirmed = False

        # Set up logging to file
        log_folder = "../../Flight Logs"
        os.makedirs(log_folder, exist_ok=True)
        start_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = os.path.join(log_folder, f"flight-{start_time}.log")

        logging.basicConfig(
            filename=log_filename,
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%H:%M:%S'
        )

    def get_center_position(self):
        dy = self.get_pitch() * PIXELS_PER_RAD
        dx = self.get_roll() * PIXELS_PER_RAD
        center_x, center_y = round(CENTERED_X + dx), round(CENTERED_Y + dy)
        return center_x, center_y

    def check_if_mode_guided(self) -> bool:
        if self.vehicle is None:
            return False
        return self.vehicle.mode == 'GUIDED'

    def log_and_print(self, message: str):
        print(message)
        logging.info(message)

    def is_armed(self):
        return self.vehicle.armed

    def connect(self):
        def status_listener(_self, name, message):
            self.log_and_print(f"[STATUS]: {message.text}")

        # Function to handle changes in the kill switch position
        def kill_switch_listener(_self, attr_name, message):
            self.log_and_print(f"Radio signal detected: {message.chan8_raw}")

            if message.chan8_raw > 1500:
                # Kill switch is ON: Enable manual control
                self.vehicle.channels.overrides = {}  # Clear all overrides
                self.vehicle.mode = VehicleMode(KILL_SWITCH_MODE)
                self.log_and_print("Kill switch activated: Manual control enabled.")
                exit(1)

        def max_alt_listener(_self, attr_name, value):
            current_altitude = value.global_relative_frame.alt
            if current_altitude is not None and current_altitude >= self.max_altitude:
                self.log_and_print(
                    f"Altitude {current_altitude}m exceeds maximum limit of {self.max_altitude}m. Initiating landing.")
                # exit(1)
                # self.vehicle.mode = VehicleMode("LAND")

        self.log_and_print('Connecting...')
        self.vehicle = connect(self.connection_string, wait_ready=True)

        # Add listeners
        self.vehicle.add_attribute_listener('RC_CHANNELS_RAW', kill_switch_listener)
        self.vehicle.add_attribute_listener('location', max_alt_listener)
        self.vehicle.add_message_listener('STATUSTEXT', status_listener)

        # wait until battery info arrives
        while self.vehicle.battery.voltage is None:
            print("Waiting for battery data…")
            time.sleep(0.5)

        # read and print voltage
        self.log_and_print(f"Battery voltage: {self.vehicle.battery.voltage:.2f} V")

        self.log_and_print('Connected!')

        self.calibrate_barometer()
    
    def calibrate_barometer(self):
        self.log_and_print("Calibrating barometer...")
         # send the command_long to reset baro “ground pressure”
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,                                        # target system, target component (0 = autopick)
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,                                           # confirmation
            0, 0, 1, 0, 0, 0, 0                          # param1…param7: only param3=1 (ground-pressure)
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        time.sleep(2)    # 2 seconds gives the baro time to settle
        self.log_and_print("Baro zeroed—ready to arm.")

    def confirm_arm(self):
        self.arm_confirmed = True

    @require_guided
    def takeoff(self):
        self.log_and_print("Taking off...")
        self.state = State.TAKEOFF

        self.vehicle.armed = True
        self.vehicle.flush()
        time.sleep(3)
        self.arm_confirmed = False
        self.log_and_print('Confirm armed: ')
        while not self.arm_confirmed:
            time.sleep(0.2)

        # while not self.vehicle.armed:
        #     print('Waiting for arm...')
        #     time.sleep(1)

        self.log_and_print("Armed!")

        self.vehicle.simple_takeoff(INITIAL_ALTITUDE)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            self.log_and_print(f"Alt is: {altitude}")
            if altitude >= 0.95 * INITIAL_ALTITUDE:
                break
            time.sleep(0.1)

        # self.vehicle.wait_simple_takeoff(INITIAL_ALTITUDE)

        self.log_and_print("In the air!!")

    def get_altitude(self):
        altitude = self.vehicle.location.global_relative_frame.alt
        return altitude

    def get_current_location(self):
        return self.vehicle.location.global_relative_frame

    def get_heading(self):
        return self.vehicle.heading

    def get_state(self):
        return self.state
    
    def get_velocity(self):
        return self.vehicle.velocity
    
    def get_yaw(self):
        return self.vehicle.attitude.yaw
        
    def get_pitch(self):
        return self.vehicle.attitude.pitch
    
    def get_roll(self):
        return self.vehicle.attitude.roll

    @require_guided
    def move_forward(self, distance):
        """
        Moves the drone forward in the direction of its current heading.
        """
        self.log_and_print(f"Moving forward {distance} meters")

        current_location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading  # Current heading in degrees (0-360)

        # Calculate the target location based on heading
        target_location = calculate_target_location(current_location, heading, distance)
        self.vehicle.simple_goto(target_location, airspeed=0.8)

        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance_to_target = get_distance_meters(current_location, target_location)

            if distance_to_target <= 0.2:  # Consider 1 meter as the acceptable threshold
                self.log_and_print("Reached target location")
                break

            time.sleep(1)

    @require_guided
    def change_altitude(self, delta: float):
        """
        Change altitude by `delta` meters (positive → ascend, negative → descend).
        """
        self.log_and_print("Changing altitude...")
        # 1. Read current altitude
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        current_alt = self.vehicle.location.global_relative_frame.alt

        # 2. Compute new target, clamp ≥ 0 and ≤ max_altitude
        target_alt = max(0.0, min(current_alt + delta, self.max_altitude))

        # 3. Command the vehicle to fly to the new altitude at current lat/lon
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, target_alt))

        # 4. Wait until we’re within `tolerance` meters of target
        while True:
            cur = self.vehicle.location.global_relative_frame.alt
            if abs(cur - target_alt) <= 0.2:
                break
            time.sleep(0.5)

    @require_guided
    def rotate(self, angle, speed_factor=0.5):
        """
        Rotate the drone by a fixed yaw angle at a specified speed factor.

        :param vehicle: Vehicle object from DroneKit
        :param yaw_angle: Desired yaw angle change in degrees
        :param speed_factor: Fraction of max yaw speed (0.0 to 1.0)
        :param relative: If True, yaw_angle is relative to current heading
        """

        current_heading = self.vehicle.heading

        new_heading = (current_heading + angle) % 360

        self.log_and_print(f"Rotating to {new_heading} degrees...")
        
        self.rotate_to(new_heading, speed_factor=speed_factor)
    
    @require_guided
    def rotate_to(self, angle, speed_factor=0.5):
        """
        Rotate the drone to a specific yaw angle.

        :param vehicle: Vehicle object from DroneKit
        :param yaw_angle: Desired yaw angle in degrees
        """
        # Get the maximum yaw rate from the drone's parameters (default 200/s if unknown)
        max_yaw_rate = self.vehicle.parameters.get('ATC_RATE_Y_MAX', 20000) / 100.0  # Convert from centidegrees/sec
        self.log_and_print(f"max turn rate: {max_yaw_rate}")
        yaw_rate = max_yaw_rate * speed_factor

        # MAVLink command to rotate the drone
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            angle,  # param 1: Yaw angle
            yaw_rate,  # param 2: Yaw rate
            0,  # param 3: Absolute heading
            0, 0, 0, 0  # param 4-7: Not used
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    @require_guided
    def set_speed(self, velocity_x: float, velocity_y: float, velocity_z: float):
        """
        Move vehicle relative to current heading and rotate to align with motion vector.

        velocity_x, velocity_y: body-frame forward/rightward velocities (m/s)
        velocity_z: body-frame downward velocity (m/s)
        """
        self.log_and_print('Setting speed to {}, {}, {} with rotation'.format(velocity_x, velocity_y, velocity_z))

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_y, velocity_x, -velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    @require_guided
    def set_speed_for_duration(self, velocity_x: float, velocity_y: float, velocity_z: float, duration_seconds: int):
        for i in range(duration_seconds):
            self.log_and_print('Movement duration in direction: {x}, {y}, {z} after {t} secs'.format(
                x=velocity_x, y=velocity_y, z=velocity_z, t=i
            ))
            self.set_speed(velocity_x, velocity_y, velocity_z)
            time.sleep(1)
    
    @require_guided
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

    @require_guided
    def old_pid(self, target_position, only_rotate=False):
        """
        Moves the drone towards the target position at a constant speed.

        First rotates towards the target if needed (1° steps), then moves when aligned.

        Coordinates are in the camera frame where the drone’s forward direction is along the +Y axis.

        Parameters:
            target_position (tuple): Target (x, y) position in Cartesian coordinates relative to the camera.
            speed (float): Absolute speed in the XY plane (default is 0.1 m/s to avoid tilt).
        """
        target_x, target_y = target_position

        # Calculate the distance to the target
        distance = math.hypot(target_x, target_y)
        # Rotate stepwise (1°) until within tolerance
        if distance > YAW_TOLERANCE_RADIUS:
            rotation_step = 5  # max degrees per rotation call
            if abs(target_x) > YAW_TOLERANCE_THRESHOLD:
                # Rotate one degree toward the target
                rotation_direction = -np.sign(target_y) * rotation_step * target_x * KP_YAW
                self.rotate(rotation_direction, speed_factor=0.1)
                return

        if only_rotate:
            return

        # Scale by the desired speed
        velocity_x = target_x * KP_V
        velocity_y = -target_y * KP_V  # Image y is upside-down

        v_norm = math.hypot(velocity_x, velocity_y)
        if v_norm > MAX_SPEED:  # make sure speed is not too high
            velocity_x /= v_norm
            velocity_y /= v_norm

        # Set the velocity in the XY plane, keeping Z velocity zero
        self.set_speed(velocity_x, velocity_y, 0.0)

    @require_guided
    def pid(self, target_position, only_rotate=False):
        """
        Moves the drone towards the target position at a constant speed.

        First rotates towards the target if needed (1° steps), then moves when aligned.

        Coordinates are in the camera frame where the drone’s forward direction is along the +Y axis.

        Parameters:
            target_position (tuple): Target (x, y) position in Cartesian coordinates relative to the camera.
            speed (float): Absolute speed in the XY plane (default is 0.1 m/s to avoid tilt).
        """
        target_x, target_y = target_position

        # Initialize and compute dt
        now = time.time()
        dt = 0
        if self._prev_time is None:
            self._prev_time = now
            return
        dt = now - self._prev_time
        self.log_and_print(f"fps: {1/dt}")
        if dt > 1:
            self.log_and_print("Resetting PID...")
            self._prev_time = now
            self._yaw_history.clear()
            self._vx_history.clear()
            self._vy_history.clear()
            self._prev_error_yaw = 0.0
            self._prev_error_vx = 0.0
            self._prev_error_vy = 0.0
            self._integral_yaw = 0.0
            self._integral_vx = 0.0
            self._integral_vy = 0.0
            self._yaw_miss = 0
            self._vel_miss = 0
            return
        self._prev_time = now

        # Compute yaw error (px)
        angle_err = target_x
        # Update yaw integral sliding window
        inc_yaw = angle_err * dt
        self._yaw_history.append(inc_yaw)
        self._integral_yaw = sum(self._yaw_history)
        # Derivative
        deriv_yaw = (angle_err - self._prev_error_yaw) / dt if dt > 0 else 0.0
        self._prev_error_yaw = angle_err

        # Distance
        distance = math.hypot(target_x, target_y)
        # Rotate stepwise
        if distance > YAW_TOLERANCE_RADIUS:
            if abs(angle_err) > YAW_TOLERANCE_THRESHOLD:
                # velocity missed
                self._vel_miss += 1
                if self._vel_miss > MISS_LIMIT:
                    self._vx_history.clear()
                    self._vy_history.clear()
                    self._prev_error_vx = 0.0
                    self._prev_error_vy = 0.0
                    self._integral_vx = 0.0
                    self._integral_vy = 0.0
                    self._vel_miss = 0
                # reset yaw miss
                self._yaw_miss = 0
                # PID-based rotation
                rot = -np.sign(target_y) * KP_YAW * angle_err + KI_YAW * self._integral_yaw + KD_YAW * deriv_yaw
                self.rotate(rot, speed_factor=0.1)
                return
        # yaw missed
        self._yaw_miss += 1
        if self._yaw_miss > MISS_LIMIT:
            self._yaw_history.clear()
            self._integral_yaw = 0.0
            self._prev_error_yaw = 0.0
            self._yaw_miss = 0

        if only_rotate:
            self._vel_miss += 1
            if self._vel_miss > MISS_LIMIT:
                self._vx_history.clear()
                self._vy_history.clear()
                self._prev_error_vx = 0.0
                self._prev_error_vy = 0.0
                self._integral_vx = 0.0
                self._integral_vy = 0.0
                self._vel_miss = 0
            return

        # Velocity PID compute
        err_vx = target_x
        err_vy = target_y
        # Update sliding windows
        inc_vx = err_vx * dt
        inc_vy = err_vy * dt
        self._vx_history.append(inc_vx)
        self._vy_history.append(inc_vy)
        self._integral_vx = sum(self._vx_history)
        self._integral_vy = sum(self._vy_history)
        # Derivatives
        deriv_vx = (err_vx - self._prev_error_vx) / dt if dt > 0 else 0.0
        deriv_vy = (err_vy - self._prev_error_vy) / dt if dt > 0 else 0.0
        self._prev_error_vx = err_vx
        self._prev_error_vy = err_vy
        # reset vel miss
        self._vel_miss = 0

        # PID outputs
        vx = KP_V * err_vx + KI_V * self._integral_vx + KD_V * deriv_vx
        vy = KP_V * err_vy + KI_V * self._integral_vy + KD_V * deriv_vy
        vy = -vy  # invert image Y

        # cap speed
        v_norm = math.hypot(vx, vy)
        if v_norm > MAX_SPEED:
            vx /= v_norm
            vy /= v_norm

        self.set_speed(vx, vy, 0.0)

    @require_guided
    def follow_path(self, waypoints: List[Waypoint], detection_obj: ImageDetection, safe=False, detect=True, stop_on_detect=True):
        """
        Follow a series of waypoints at a constant speed.

        Parameters:
            waypoints (List[Waypoint]): List of Waypoint instances.
            detection_obj (ImageDetection): detection object to integrate into the flight loop.
        """
        for wp in waypoints:
            if wp.movement_action == MovementAction.MOVEMENT:
                self.log_and_print(f"Moving to location: {wp.position}")
                if safe:
                    dist = get_distance_meters(
                            self.get_current_location(), wp.position
                        )
                    self.log_and_print(f"Moving {dist} meters...")
                    input("Press entet to confirm movement...")
                self.vehicle.simple_goto(wp.position, airspeed=0.8)

                while True:
                    if detect:
                        if detection_obj.image_detection_data['position'] != (None, None):
                            self.log_and_print("Found target!!")
                            if stop_on_detect:
                                self.stop_movement()
                                return True
                        time.sleep(0.02)  # small pause
                    current_location = self.vehicle.location.global_relative_frame
                    distance_to_target = get_distance_meters(current_location, wp.position)

                    if distance_to_target <= 0.2:  # Consider 0.2m as acceptable threshold
                        self.log_and_print("Reached target location")
                        break

                    time.sleep(0.2)

            elif wp.movement_action == MovementAction.ROTATION:
                self.log_and_print(f"Rotating to angle: {wp.angle}")
                self.rotate_to(wp.angle)
                time.sleep(1)  # Wait for rotation to complete
                if detect:
                    start = time.time()
                    while time.time() - start < 0.5:
                        if detection_obj.image_detection_data['position'] != (None, None):
                            self.log_and_print("Found target!!")
                            if stop_on_detect:
                                self.stop_movement()
                                return True
                        time.sleep(0.02)  # small pause

            time.sleep(1)
        
        return False

    def has_stopped(self, error_tolerence=0.1):
        return abs(self.vehicle.groundspeed) < error_tolerence

    @require_guided
    def stop_movement(self):
        self.set_speed(0, 0, 0)
        self.state = State.ON_TARGET

    def is_on_target(self, target_position: Tuple[int, int], error_tolerence_raduis=DROP_RADIUS):
        return target_position[0] ** 2 + target_position[1] ** 2 < error_tolerence_raduis**2

    @require_guided
    def face_target(self, target_position):
        direction = calculate_direction(target_position)
        self.log_and_print(f"Rotating {direction} degrees...")
        self.rotate(direction)

    @require_guided
    def return_to_launch(self):
        self.log_and_print("Returning home!")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(1)

    @require_guided
    def land(self):
        self.log_and_print("Landing! [DO NOT KILL THE SCRIPT!]")
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(10)

    def distance_from_home(self):
        # Get the home location
        if self.vehicle.gps_0.eph is None:
            return 0
        home_location = self.vehicle.home_location
        current_location = self.vehicle.location.global_relative_frame
        dist = get_distance_meters(home_location, current_location)
        return dist
    
    def mission_completed(self):
        return self.done

    @require_guided
    def assassinate(self): # TODO implement servo operation
        self.done = True

    def disconnect(self):
        # wait until battery info arrives
        while self.vehicle.battery.voltage is None:
            print("Waiting for battery data…")
            time.sleep(0.5)

        # read and print voltage
        self.log_and_print(f"Battery voltage: {self.vehicle.battery.voltage:.2f} V")
        self.log_and_print("Disconnecting.")
        self.vehicle.close()

    def get_vehicle_mode(self):
        return self.vehicle.mode

    def get_battery_voltage(self):
        return self.vehicle.battery.voltage

    def reboot_pixhawk(self):
        """
        Sends a soft reboot command to the Pixhawk via MAVLink.
        Requires the script to be run with appropriate permissions.
        """
        print("Sending reboot command to Pixhawk...")

        # # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
        # # param1 = 1 → reboot autopilot (but not autopilot+companion computer)
        # self.vehicle._master.mav.command_long_send(
        #     self.vehicle._master.target_system,
        #     self.vehicle._master.target_component,
        #     mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        #     0,  # confirmation
        #     1,  # param1: 1=reboot autopilot
        #     0, 0, 0, 0, 0, 0  # unused params
        # )
        # self.log_and_print("Reboot command sent. Waiting for reboot...")

        self.vehicle.reboot()

    def set_safety_button(self, safety: bool):
        self.log_and_print(f"[INFO] {'Enabling' if safety else 'Disabling'} safety switch...")

        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SAFETY,
            0,
            int(safety),  # param1: safety enabled (1) or disabled (0)
            0, 0, 0, 0, 0, 0
        )
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


