import collections.abc
import time
from abc import ABC, abstractmethod
from typing import Tuple, List

import cv2

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative
from dataclasses import dataclass
from enum import Enum


# Define a strong-typed action enum
class MovementAction(Enum):
    MOVEMENT = "movement"
    ROTATION = "rotation"


@dataclass
class Waypoint:
    position: LocationGlobalRelative
    angle: float
    movement_action: MovementAction


class Source(ABC):
    def __init__(self, rotation=0):
        """Initialize the source with a simple rotation setting: 0, 90, 180, or 270 degrees."""
        self.rotation = rotation

    @abstractmethod
    def _get_current_frame(self):
        """This method should be implemented by subclasses to get the frame."""
        pass

    def get_current_frame(self):
        """Get the current frame, rotate it based on the rotation setting, and return it."""
        frame = self._get_current_frame()

        if frame is not None:
            # Apply the rotation
            frame = self.apply_rotation(frame)

        return frame

    def apply_rotation(self, frame):
        """Rotate the frame based on the specified rotation angle (only 90-degree increments)."""
        if self.rotation == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame  # No rotation if 0 degrees


class ImageDetection(ABC):
    def __init__(self):
        self.image_detection_data = {}

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
    def set_speed_for_duration(self, velocity_x: float, velocity_y: float, velocity_z: float, duration_seconds: int):
        pass

    @abstractmethod
    def set_speed(self, velocity_x: float, velocity_y: float, velocity_z: float):
        pass

    @abstractmethod
    def check_if_mode_guided(self) -> bool:
        pass

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def get_current_location(self):
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

    @abstractmethod
    def get_heading(self):
        pass

    @abstractmethod
    def follow_path(self, waypoints: List[Waypoint], source_obj: Source, detection_obj: ImageDetection):
        pass

    @abstractmethod
    def pid(self, target_position):
        pass

    @abstractmethod
    def has_stopped(self):
        pass

    @abstractmethod
    def stop_movement(self):
        pass

    @abstractmethod
    def is_on_target(self, target_position: Tuple[int, int]):
        pass

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

    @abstractmethod
    def move_forward(self, distance):
        pass

    @abstractmethod
    def log_and_print(self, message: str):
        pass

    @abstractmethod
    def rotate(self, angle):
        pass

    @abstractmethod
    def change_altitude(self, delta):
        pass

    @abstractmethod
    def mission_completed(self):
        pass

    @abstractmethod
    def assassinate(self):
        pass

    @abstractmethod
    def get_vehicle_mode(self):
        pass

    @abstractmethod
    def get_battery_voltage(self):
        pass

    @abstractmethod
    def get_state(self):
        pass

    @abstractmethod
    def get_velocity(self):
        pass

    @abstractmethod
    def get_yaw(self):
        pass

    @abstractmethod
    def get_pitch(self):
        pass

    @abstractmethod
    def get_roll(self):
        pass

    @abstractmethod
    def get_center_position(self):
        pass


class DroneAlgorithm(ABC):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    @abstractmethod
    def _main(self):
        pass

    def main(self):
        try:
            self._main()
        except Exception as e:
            self.drone_client.log_and_print("The code has encountered an error:")
            self.drone_client.log_and_print(e)
            self.drone_client.log_and_print("Waiting 5 seconds then landing:")
            time.sleep(5)
            self.drone_client.land()
            self.drone_client.disconnect()


class Servo(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def drop(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def get_bombs_left(self):
        pass


class VideoSaver(ABC):
    @abstractmethod
    def write_frame(self, frame):
        pass

    @abstractmethod
    def save_and_close(self):
        pass


class GUI(ABC):
    def __init__(self, drone_client: DroneClient, video_saver: VideoSaver, image_detection: ImageDetection,
                 servo: Servo,
                 enable_display=True):
        self.drone_client = drone_client
        self.video_saver = video_saver
        self.enable_display = enable_display
        self.image_detection = image_detection
        self.servo = servo
        self.enable_display = enable_display

    @abstractmethod
    def draw_gui(self, frame):
        pass

    def close(self):
        if self.video_saver is not None:
            self.video_saver.save_and_close()
        if self.enable_display:
            # destroy all OpenCV windows
            cv2.destroyAllWindows()


class Mapper(ABC):
    def __init__(self, locations: List[LocationGlobalRelative]):
        self.locations = locations

    def get_map(self, current_loc: LocationGlobalRelative):
        pass
