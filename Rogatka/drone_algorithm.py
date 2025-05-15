import collections.abc

from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm, Waypoint, MovementAction, GUI
from Rogatka.servo_motor import ServoMotor
from .utils import calculate_target_location

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative
from typing import List
import threading
import time
import numpy as np

from BirdBrain.settings import (START_LAT,
                                START_LON,
                                LONG_SEGMENT,
                                SHORT_SEGMENT,
                                STEPS,
                                INITIAL_ANGLE,
                                RIGHT_ANGLE,
                                LEFT_ANGLE,
                                CENTERED_X,
                                CENTERED_Y,
                                RE_SEARCH_LIMIT, INITIAL_ALTITUDE)


class MainDroneAlgorithm(DroneAlgorithm):
    def __init__(self, img_detection: ImageDetection, source: Source,
                 drone_client: DroneClient, servo: ServoMotor):
        super().__init__(drone_client)
        self.source = source
        self.img_detection = img_detection
        self.servo = servo
        self.last_relative_pos = None
        
    def generate_new_path(self) -> List[Waypoint]:
        waypoints: List[Waypoint] = [
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, INITIAL_ALTITUDE),
                angle=INITIAL_ANGLE,
                movement_action=MovementAction.MOVEMENT
            ),
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, INITIAL_ALTITUDE),
                angle=INITIAL_ANGLE,
                movement_action=MovementAction.ROTATION
            )
        ]
        lengths = [SHORT_SEGMENT / 2, LONG_SEGMENT / 2, SHORT_SEGMENT, LONG_SEGMENT, SHORT_SEGMENT,
                   LONG_SEGMENT / 2, SHORT_SEGMENT / 2]
        for length in lengths:
            last = waypoints[-1]
            # Move short segment
            waypoints.append(
                Waypoint(
                    position=calculate_target_location(last.position, last.angle, length),
                    angle=last.angle,
                    movement_action=MovementAction.MOVEMENT
                )
            )

            turn_angle = (last.angle + RIGHT_ANGLE) % 360
            waypoints.append(
                Waypoint(
                    position=waypoints[-1].position,
                    angle=turn_angle,
                    movement_action=MovementAction.ROTATION
                )
            )

        return waypoints[:-1]  # remove last right rotation
    
    def generate_path(self, steps=STEPS) -> List[Waypoint]:
        waypoints: List[Waypoint] = [
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, INITIAL_ALTITUDE),
                angle=INITIAL_ANGLE,
                movement_action=MovementAction.MOVEMENT
            ),
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, INITIAL_ALTITUDE),
                angle=INITIAL_ANGLE,
                movement_action=MovementAction.ROTATION
            )
        ]

        for i in range(steps):
            last = waypoints[-1]
            # Move short segment
            waypoints.append(
                Waypoint(
                    position=calculate_target_location(last.position, last.angle, SHORT_SEGMENT),
                    angle=last.angle,
                    movement_action=MovementAction.MOVEMENT
                )
            )

            # Rotate: right on even i, left on odd i
            turn_angle = RIGHT_ANGLE if i % 2 == 0 else LEFT_ANGLE
            waypoints.append(
                Waypoint(
                    position=waypoints[-1].position,
                    angle=turn_angle,
                    movement_action=MovementAction.ROTATION
                )
            )

            # Move long segment
            last = waypoints[-1]
            waypoints.append(
                Waypoint(
                    position=calculate_target_location(last.position, last.angle, LONG_SEGMENT),
                    angle=last.angle,
                    movement_action=MovementAction.MOVEMENT
                )
            )

            # Face initial direction
            waypoints.append(
                Waypoint(
                    position=waypoints[-1].position,
                    angle=INITIAL_ANGLE,
                    movement_action=MovementAction.ROTATION
                )
            )

        return waypoints

    def perform_search_pattern(self, stop_on_detect=True):
        waypoints = self.generate_new_path()
        return self.drone_client.follow_path(waypoints, self.img_detection, stop_on_detect=stop_on_detect)

    def assassinate(self):
        # self.drone_client.assassinate()
        self.servo.drop()
        self.drone_client.log_and_print('DROPPING PAYLOAD!!')

    def re_search(self, direction: int=1):
        self.drone_client.log_and_print("Re-Searching target...")
        for _ in range(72):
            self.drone_client.rotate(direction * 10, speed_factor=0.8)
            if self.img_detection.image_detection_data['position'] != (None, None):
                self.drone_client.log_and_print("Re-Found target!!")
                return True
            time.sleep(0.02)  # small pause
        return False

    def _main(self, search=True, only_search=False, stop_on_detect=True, only_rotate=False):
        self.drone_client.takeoff()

        if search:
            self.perform_search_pattern(stop_on_detect=stop_on_detect)

        self.drone_client.log_and_print("Finished search! Continuing mission...")
        if only_search:
            return

        failed_frames = 0

        while not self.drone_client.mission_completed():
            target_position = self.img_detection.image_detection_data['position']
            if target_position == (None, None):
                failed_frames += 1
                if failed_frames >= RE_SEARCH_LIMIT:
                    if self.last_relative_pos is not None:
                        direction = 1 if self.last_relative_pos[0] > 0 else -1
                    else:
                        direction = 1
                    res = self.re_search(direction=direction)
                    if not res:
                        self.drone_client.log_and_print("Failed to relocate target position! Aborting mission...")
                        break
                continue

            failed_frames = 0
            center_position = self.drone_client.get_center_position()
            relative_position = target_position[0] - center_position[0], target_position[1] - center_position[1]
            self.last_relative_pos = relative_position

            self.drone_client.log_and_print("Found target position in frame!")
            if self.drone_client.is_on_target(relative_position):
                self.assassinate()
            else:
                self.drone_client.pid(relative_position, only_rotate=only_rotate)

        self.drone_client.land()

    def just_rotate(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        self.drone_client.log_and_print("Looking for target...")

        while True:
            target_position = self.img_detection.locate_target(
                self.source.get_current_frame())  # position is in pixels relative to the desired target point
            if target_position != (None, None):
                self.drone_client.log_and_print(f"Facing target at: {target_position}")
                self.drone_client.pid(target_position, only_rotate=True)

        self.drone_client.land()
        self.drone_client.disconnect()

    def main(self):
        try:
            self._main()
        except Exception as e:
            self.drone_client.log_and_print("The code has encountered an error:")
            self.drone_client.log_and_print(e)
            self.drone_client.log_and_print("Waiting 5 seconds then landing:")
            time.sleep(5)
            self.drone_client.land()
        finally:
            self.gui.close()
            self.drone_client.disconnect()
