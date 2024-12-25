from VideoSource import VideoSource
from ImageDetectionModel import ImageDetectionModel
from ImageProcessingConstants import *


def main():
    video_source = VideoSource(VIDEO_PATH, START_FROM_SECONDS)
    image_detection_model = ImageDetectionModel(REFERENCE_IMAGE_PATH,video_source)
    done = False
    while not done:
        target_position = image_detection_model.process_next_frame()
        print(target_position)
        # if self.drone_client.is_on_target(target_position):
        #     if self.drone_client.has_stopped():
        #         self.assassinate()
        #     else:
        #         self.drone_client.stop_movement()
        # else:
        #     self.drone_client.goto_target(target_position)


if __name__ == "__main__":
    main()
