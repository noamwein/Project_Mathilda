import argparse
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from EagleEye.ImageDetectionModel import ImageDetectionModel
from EagleEye.ImageProcessingConstants import *
from EagleEye.VideoSource import VideoSource


def main(video_path, image_path, display, always_recognize_person):
    video_source = VideoSource(video_path, START_FROM_SECONDS)
    image_detection_model = ImageDetectionModel(image_path, video_source, display=display,
                                                always_recognize_person=always_recognize_person)
    done = False
    while not done:
        target_position = image_detection_model.locate_target(video_source.get_current_frame())
        print(target_position)
        # Uncomment and implement drone client logic if needed
        # if self.drone_client.is_on_target(target_position):
        #     if self.drone_client.has_stopped():
        #         self.assassinate()
        #     else:
        #         self.drone_client.stop_movement()
        # else:
        #     self.drone_client.goto_target(target_position)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Image Detection Script")
    parser.add_argument(
        '--video-path',
        type=str,
        default=None,
        help="Path to the video file."
    )
    parser.add_argument(
        '--image-path',
        type=str,
        default=None,
        help="Path to the reference image."
    )
    parser.add_argument(
        '--disable-display',
        action='store_true',
        help="Disable to display the video feed."
    )
    parser.add_argument(
        '--recognize',
        action='store_true',
        help="Always try to recognize the person in the frame (for testing face recognition)."
    )

    args = parser.parse_args()
    main(video_path=args.video_path, image_path=args.image_path, display=not args.disable_display,
         always_recognize_person=args.recognize)
