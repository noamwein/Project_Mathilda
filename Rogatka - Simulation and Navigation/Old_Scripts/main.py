from VideoSource import VideoSource
from ImageDetectionModel import ImageDetectionModel
from drone_client import BasicClient
from drone_algorithm import DroneAlgorithm
import logging

from ImageProcessingConstants import *

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

