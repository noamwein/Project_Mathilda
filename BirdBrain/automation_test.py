import logging
import os
import sys
import argparse

# allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from Rogatka.test_algorithms import (
    TestAlgorithm1,
    TestAlgorithm2,
    TestAlgorithm3,
    TestAlgorithm4,
    TestAlgorithm5,
    TestAlgorithm6,
    TestAlgorithm7,
    TestAlgorithm8,
    TestAlgorithm9,
    TestAlgorithm10,
    TestAlgorithm11,
    TestAlgorithm12,
    TestAlgorithm13
)
from Rogatka.drone_client import BasicClient


def main():
    parser = argparse.ArgumentParser(
        description="Run a specified test algorithm (1-6)"
    )
    parser.add_argument(
        "algorithm",
        type=int,
        choices=range(1, 14),
        help="Test algorithm number (1-13) to execute",
    )
    args = parser.parse_args()

    # map numbers to algorithm classes
    algo_map = {
        1: TestAlgorithm1,
        2: TestAlgorithm2,
        3: TestAlgorithm3,
        4: TestAlgorithm4,
        5: TestAlgorithm5,
        6: TestAlgorithm6,
        7: TestAlgorithm7,
        8: TestAlgorithm8,
        9: TestAlgorithm9,
        10: TestAlgorithm10,
        11: TestAlgorithm11,
        12: TestAlgorithm12,
        13: TestAlgorithm13
    }
    selected_algo_cls = algo_map[args.algorithm]

    # instantiate the selected algorithm with BasicClient
    main_alg = selected_algo_cls(
        BasicClient(
            '/dev/ttyACM0',  # serial port
            1.5,             # initial altitude
            10,              # max altitude
            20,              # min battery percent
            logger=logging.getLogger(__name__)
        )
    )

    main_alg.main()


if __name__ == "__main__":
    main()
