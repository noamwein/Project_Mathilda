import collections.abc

from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm, Waypoint, MovementAction, GUI
from Rogatka.servo_motor import ServoMotor
from .utils import calculate_target_location

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative
from typing import List
import threading
import time

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
                                RE_SEARCH_LIMIT)


class MainDroneAlgorithm(DroneAlgorithm):
    def __init__(self, img_detection: ImageDetection, source: Source,
                 drone_client: DroneClient, servo: ServoMotor, gui: GUI):
        super().__init__(drone_client)
        self.source = source
        self.img_detection = img_detection
        self.servo = servo
        self.gui = gui

    def generate_path(self, steps=STEPS) -> List[Waypoint]:
        initial_alt = self.drone_client.get_initial_altitude()
        waypoints: List[Waypoint] = [
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, initial_alt),
                angle=INITIAL_ANGLE,
                movement_action=MovementAction.MOVEMENT
            ),
            Waypoint(
                position=LocationGlobalRelative(START_LAT, START_LON, initial_alt),
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
        waypoints = self.generate_path()
        return self.drone_client.follow_path(waypoints, self.source, self.img_detection, stop_on_detect=stop_on_detect)

    def search_with_preview(self, search, stop_on_detect):
        def search_thread():
            self.drone_client.connect()
            self.drone_client.takeoff()

            if search:
                self.perform_search_pattern(stop_on_detect=stop_on_detect)

        thread = threading.Thread(target=search_thread)
        thread.start()

        while thread.is_alive():
            frame = self.source.get_current_frame()
            self.img_detection.locate_target(frame)
            if self.gui is not None:
                self.gui.draw_gui(frame)

    def assassinate(self):
        # self.drone_client.assassinate()
        self.servo.drop()
        self.drone_client.log_and_print('DROPPING PAYLOAD!!')

    def re_search(self):
        self.drone_client.log_and_print("Re-Searching target...")
        for _ in range(360 / 5):
            self.drone_client.rotate(5)
            frame = self.source.get_current_frame()
            if self.gui is not None:
                self.gui.draw_gui(frame)
            if self.img_detection.detect_target(frame):
                self.drone_client.log_and_print("Re-Found target!!")
                return True
            time.sleep(0.02)  # small pause
        return False

    def _main(self, search=True, only_search=False, stop_on_detect=True, only_rotate=False):
        self.search_with_preview(search=search, stop_on_detect=stop_on_detect)

        self.drone_client.log_and_print("Finished search! Continuing mission...")
        if only_search:
            return

        failed_frames = 0

        try:
            while not self.drone_client.mission_completed():
                frame = self.source.get_current_frame()
                target_position = self.img_detection.locate_target(
                    frame)  # position is in pixels relative to the desired target point
                if self.gui is not None:
                    self.gui.draw_gui(frame)

                if target_position == (None, None):
                    failed_frames += 1
                    if failed_frames >= RE_SEARCH_LIMIT:
                        res = self.re_search()
                        if not res:
                            self.drone_client.log_and_print("Failed to relocate target position! Aborting mission...")
                            break
                    continue
                else:
                    failed_frames = 0

                center_position = self.drone_client.get_center_position()
                relative_position = target_position[0] - center_position[0], target_position[1] - center_position[1]

                self.drone_client.log_and_print("Found target position in frame!")
                if self.drone_client.is_on_target(relative_position):
                    self.assassinate()
                else:
                    self.drone_client.pid(relative_position, only_rotate=only_rotate)

        finally:
            self.gui.video_saver.save_and_close()
        self.drone_client.land()
        self.drone_client.disconnect()

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
