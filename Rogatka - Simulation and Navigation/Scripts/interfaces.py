from typing import Tuple


class Source:
    def get_current_frame(self):
        pass


class ImageDetection:
    def find_target(self, frame) -> Tuple[int, int]:
        pass


class DroneClient:
    def connect(self):
        pass

    def takeoff(self):
        pass

    def return_to_launch(self):
        pass

    def disconnect(self):
        pass


class DroneAlgorithm:
    def __init__(self, img_detection: ImageDetection, source: Source,
                 drone_client: DroneClient):
        self.source = source
        self.img_detection = img_detection
        self.drone_client = drone_client
    
    def mission_completed(self):
        pass

    def goto_target(self, target_position: Tuple[int, int]):
        pass

    def is_on_target(self, target_position: Tuple[int, int]):
        pass

    def assassinate(self):
        pass

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        
        while not self.mission_completed():
            frame = self.source.get_current_frame()
            target_position = self.img_detection.find_target(frame=frame)

            if self.is_on_target(target_position):
                self.assassinate()
            else:
                self.goto_target(target_position)

        
        self.drone_client.return_to_launch()
        self.drone_client.disconnect()
    