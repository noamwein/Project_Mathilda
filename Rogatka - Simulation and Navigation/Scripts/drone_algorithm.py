from interfaces import Source, ImageDetection, DroneClient
from typing import Tuple

class DroneAlgorithm:
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

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        
        while not self.mission_completed():
            frame = self.source.get_current_frame()
            target_position = self.img_detection.locate_traget(frame=frame)

            if self.drone_client.is_on_target(target_position):
                if self.drone_client.has_stopped():
                    self.assassinate()
                else:
                    self.drone_client.stop_movement()
            else:
                self.drone_client.goto_target(target_position)

        
        self.drone_client.return_to_launch()
        self.drone_client.disconnect()