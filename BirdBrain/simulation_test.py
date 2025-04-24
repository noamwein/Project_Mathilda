import logging
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from Rogatka.drone_algorithm import MainDroneAlgorithm
from Rogatka.drone_client import BasicClient
from EagleEye.image_detection_models.dummy_img_detection import Dummy_detector
from EagleEye.sources.dummy_source import Dummy_source


def main():
    main_alg = MainDroneAlgorithm(
        Dummy_detector(),
        Dummy_source(),
        BasicClient(
            'tcp:127.0.0.1:5763',
            3,
            10,
            20,
            logger=logging.getLogger(__name__))
    )

    main_alg.main()


if __name__ == "__main__":
    main()
