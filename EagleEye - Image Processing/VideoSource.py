from abc import ABC, abstractmethod

class Source(ABC):
    @abstractmethod
    def get_current_frame(self):
        pass