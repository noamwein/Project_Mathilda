from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm
from .utils import get_distance_meters, calculate_target_location
from dronekit import LocationGlobalRelative

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
        self.done = False

    def mission_completed(self):
        return self.done

    def assassinate(self):
        self.done = True
    
    @property
    def frame(self):
        return self.source.get_current_frame()
    
    def generate_path(self):
        # Starting Position and Heading
        waypoints = [
            (
                LocationGlobalRelative(START_LAT, 
                                       START_LON, 
                                       self.drone_client.get_initial_altitude()), 
                INITIAL_ANGLE, 
                "movement"
            ),
            (
               LocationGlobalRelative(START_LAT, 
                                       START_LON, 
                                       self.drone_client.get_initial_altitude()), 
                INITIAL_ANGLE, 
                "rotation" 
            )
        ]

        for i in range(STEPS):
            # Move short
            waypoints.append(
                (
                    calculate_target_location(waypoints[-1][0], 
                                              waypoints[-1][1], 
                                              SHORT_SEGMENT
                                             ),
                    waypoints[-1][1], 
                    "movement"
                )
            )

            # Turn right (even i) or left (odd i)
            angle = RIGHT_ANGLE if i % 2 == 0 else LEFT_ANGLE
            waypoints.append(
                (
                    waypoints[-1][0], 
                    angle, 
                    "rotation"
                )
            )

            # Move long
            waypoints.append(
                (
                    calculate_target_location(waypoints[-1][0], 
                                              waypoints[-1][1], 
                                              LONG_SEGMENT
                                             ), 
                    waypoints[-1][1], 
                    "movement"
                )
            )

            # Face front
            waypoints.append(
                (
                    waypoints[-1][0], 
                    INITIAL_ANGLE, 
                    "rotation"))
        
        return waypoints
    
    def perform_search_pattern(self):
        waypoints = self.generate_path()
        self.drone_client.follow_path(waypoints, self.img_detection)

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.perform_search_pattern()

        # TODO implement assasination
        # while not self.mission_completed() and not self.drone_client.mission_terminated():
        #     target_position = self.img_detection.locate_target(self.frame)
        #     if target_position != (None, None):
        #         if self.drone_client.is_on_target(target_position):
        #             if self.drone_client.has_stopped():
        #                 self.assassinate()
        #             else:
        #                 self.drone_client.stop_movement()
        #         else:
        #             self.drone_client.goto_target(target_position)

        self.drone_client.land()
        self.drone_client.disconnect()
