import logging
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from Rogatka.testing.test_algorithms import TestAlgorithm1
from Rogatka.drone_client import BasicClient


def main():
    main_alg = TestAlgorithm1(
        BasicClient(
            '/dev/ttyACM0',
            3,
            10,
            20,
            logger=logging.getLogger(__name__))
    )

    main_alg.main()


if __name__ == "__main__":
    main()
