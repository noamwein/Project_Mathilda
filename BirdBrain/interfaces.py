import time
from abc import ABC, abstractmethod
from typing import Tuple


class Source(ABC):
    @abstractmethod
    def get_current_frame(self):
        pass


class ImageDetection(ABC):
    @abstractmethod
    def detect_target(self, frame) -> bool:
        pass

    @abstractmethod
    def locate_target(self, frame) -> Tuple[int, int]:
        pass


def require_guided(func):
    def wrapper(self: DroneClient, *args, **kwargs):
        while not self.check_if_mode_guided():
            print('Waiting for guided...')
            time.sleep(0.5)
        func(self, *args, **kwargs)
    return wrapper


class DroneClient(ABC):
    @abstractmethod
    def check_if_mode_guided(self) -> bool:
        pass

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def is_armed(self):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def get_altitude(self):
        pass
    #
    # @abstractmethod
    # def goto_target(self, target_position: Tuple[int, int]):
    #     pass

    # @abstractmethod
    # def goto_fast(self, target_position):
    #     pass
    #
    # @abstractmethod
    # def has_stopped(self):
    #     pass
    #
    # @abstractmethod
    # def stop_movement(self):
    #     pass
    #
    # @abstractmethod
    # def is_on_target(self, target_position: Tuple[int, int]):
    #     pass

    @abstractmethod
    def distance_from_home(self) -> float:
        pass

    @abstractmethod
    def return_to_launch(self):
        pass

    @abstractmethod
    def land(self):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    # @abstractmethod
    # def move_forward(self, distance):
    #     pass

    @abstractmethod
    def rotate(self, angle):
        pass

    @abstractmethod
    def mission_terminated(self) -> bool:
        pass


class DroneAlgorithm(ABC):
    @abstractmethod
    def main(self):
        pass
