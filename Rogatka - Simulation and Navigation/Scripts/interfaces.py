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
    def locate_traget(self, frame) -> Tuple[int, int]:
        pass


class DroneClient(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def takeoff(self):
        pass

    @abstractmethod
    def return_to_launch(self):
        pass

    @abstractmethod
    def disconnect(self):
        pass
    