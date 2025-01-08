import argparse
import logging
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from EagleEye.ImageDetectionModel import ImageDetectionModel
from EagleEye.ImageProcessingConstants import *
from EagleEye.VideoSource import VideoSource
from Rogatka.drone_algorithm import DroneAlgorithm
from Rogatka.drone_client import BasicClient


def main(video_path, image_path, display):
    video_source = VideoSource(video_path, START_FROM_SECONDS)
    main_alg = DroneAlgorithm(
        ImageDetectionModel(image_path, video_source, display=display),
        video_source,
        BasicClient(
            'tcp:127.0.0.1:5763',
            3,
            10,
            20,
            logger=logging.getLogger(__name__))
    )

    main_alg.main()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Algorithm Script")
    parser.add_argument(
        '--video-path',
        type=str,
        default=VIDEO_PATH,
        help="Path to the video file."
    )
    parser.add_argument(
        '--image-path',
        type=str,
        default=REFERENCE_IMAGE_PATH,
        help="Path to the reference image."
    )
    parser.add_argument(
        '--disable-display',
        action='store_true',
        help="Disable to display the video feed."
    )

    args = parser.parse_args()
    main(video_path=args.video_path, image_path=args.image_path, display=not args.disable_display)
