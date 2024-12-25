from testing.dummy_img_detection import Dummy_detector
from testing.dummy_source import Dummy_source
from drone_client import BasicClient
from drone_algorithm import DroneAlgorithm
import logging

def main():
    main_alg = DroneAlgorithm(
        Dummy_detector(), 
        Dummy_source(),
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
