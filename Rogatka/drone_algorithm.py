from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm, Waypoint, MovementAction
from .utils import get_distance_meters, calculate_target_location
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
        self.source = source
        self.img_detection = img_detection
        self.drone_client = drone_client
    
    @property
    def frame(self):
        return self.source.get_current_frame()
    
    def generate_path(self) -> List[Waypoint]:
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

        for i in range(STEPS):
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

    def perform_search_pattern(self):
        waypoints = self.generate_path()
        self.drone_client.follow_path(waypoints, self.source, self.img_detection)

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        if self.perform_search_pattern(): # If target was found
            self.drone_client.log_and_print("Found target! Continuing mission...")
            while not self.drone_client.mission_completed():
                target_position = self.img_detection.locate_target(self.frame) # position is in pixels relative to the desired target point
                if target_position != (None, None):
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
