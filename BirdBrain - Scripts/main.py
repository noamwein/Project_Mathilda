from EagleEye.VideoSource import VideoSource
from EagleEye.ImageDetectionModel import ImageDetectionModel
from EagleEye.ImageProcessingConstants import *

from Rogatka.drone_client import BasicClient
from Rogatka.drone_algorithm import DroneAlgorithm

import logging

def main():
    video_source = VideoSource(VIDEO_PATH, START_FROM_SECONDS)
    main_alg = DroneAlgorithm(
        ImageDetectionModel(REFERENCE_IMAGE_PATH, video_source), 
        video_source,
        BasicClient(
            'tcp:127.0.0.1:5763', 
            3, 
            10, 
            20, 
            logger = logging.getLogger(__name__))
        )
    
    main_alg.main()

if __name__ == "__main__":
    main()

