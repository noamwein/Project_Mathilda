from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm, Waypoint, MovementAction
from .utils import get_distance_meters, calculate_target_location
import time
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative
from typing import List

# Southwest corner of the search area.
START_LAT  = 31.76953
START_LON = 35.19831
LONG_SEGMENT = 3
SHORT_SEGMENT = 2 
TURN_ANGLE = 90
STEPS = 3  # number of snake segments
INITIAL_ANGLE = 0
RIGHT_ANGLE = (INITIAL_ANGLE + TURN_ANGLE) % 360
LEFT_ANGLE = (INITIAL_ANGLE - TURN_ANGLE) % 360



class MainDroneAlgorithm(DroneAlgorithm):
    def __init__(self, img_detection: ImageDetection, source: Source,
                 drone_client: DroneClient):
        super().__init__(drone_client)
        self.source = source
        self.img_detection = img_detection
    
    @property
    def frame(self):
        return self.source.get_current_frame()
    
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
        self.drone_client.follow_path(waypoints, self.source, self.img_detection, stop_on_detect=stop_on_detect)

    def _main(self, search=True, only_search=False, stop_on_detect=True):
        self.drone_client.connect()
        self.drone_client.takeoff()

        if search:
            target_found = self.perform_search_pattern(stop_on_detect=stop_on_detect)
        else:
            target_found = True

        if target_found: 
            self.drone_client.log_and_print("Finished search! Continuing mission...")
            if not only_search:
                while not self.drone_client.mission_completed():
                    target_position = self.img_detection.locate_target(self.frame) # position is in pixels relative to the desired target point
                    if target_position != (None, None):
                        self.drone_client.log_and_print("Found target position in frame!")
                        if self.drone_client.is_on_target(target_position):
                            if self.drone_client.has_stopped():
                                self.drone_client.assassinate()
                            else:
                                self.drone_client.stop_movement()
                        else:
                            self.drone_client.pid(target_position)
        else:
            self.drone_client.log_and_print("Failed to find target. Landing...")

        self.drone_client.land()
        self.drone_client.disconnect()
    
    
    def just_rotate(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        self.drone_client.log_and_print("Looking for target...")

        while True:
            target_position = self.img_detection.locate_target(self.frame) # position is in pixels relative to the desired target point
            if target_position != (None, None):
                self.drone_client.log_and_print("Facing target at:", target_position)
                self.drone_client.pid(target_position, only_rotate=True)
        
        self.drone_client.land()
        self.drone_client.disconnect()

