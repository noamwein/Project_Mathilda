from BirdBrain.interfaces import Source, ImageDetection, DroneClient, DroneAlgorithm
import time


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

    def main(self):
        self.drone_client.connect()
        # self.drone_client.takeoff()

        while not self.drone_client.is_armed():
            time.sleep(1)

        while self.drone_client.is_armed():
            print("Looking for target...")
            res = self.img_detection.locate_target(self.frame)
            while None in res:
                time.sleep(0.1)
                if not self.drone_client.is_armed():
                    break
                res = self.img_detection.locate_target(self.frame)
            
            if not self.drone_client.is_armed():
                break
            print("Found target! Printing coordinates...")

            res = self.img_detection.locate_target(self.frame)
            while None not in res:
                print("Target coordinates:", res)
                if not self.drone_client.is_armed():
                    break
                res = self.img_detection.locate_target(self.frame)

        # while not self.mission_completed() and not self.drone_client.mission_terminated():
        #     frame = self.source.get_current_frame()
        #     target_position = self.img_detection.locate_target(self.frame)
        #     if target_position != (None, None):
        #         if self.drone_client.is_on_target(target_position):
        #             if self.drone_client.has_stopped():
        #                 self.assassinate()
        #             else:
        #                 self.drone_client.stop_movement()
        #         else:
        #             self.drone_client.goto_fast(target_position)

        # self.drone_client.return_to_launch()
        self.drone_client.disconnect()
