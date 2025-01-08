import logging

from Rogatka.drone_algorithm import DroneAlgorithm
from Rogatka.drone_client import BasicClient
from Rogatka.testing.dummy_img_detection import Dummy_detector
from Rogatka.testing.dummy_source import Dummy_source


def main():
    main_alg = DroneAlgorithm(
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
