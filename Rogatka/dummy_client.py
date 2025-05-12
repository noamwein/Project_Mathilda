import random

from BirdBrain.interfaces import DroneClient
from typing import List, Tuple
from BirdBrain.interfaces import Waypoint, Source, ImageDetection


class DummyClient(DroneClient):
    def set_speed_for_duration(self, velocity_x: float, velocity_y: float, velocity_z: float, duration_seconds: int):
        pass

    def set_speed(self, velocity_x: float, velocity_y: float, velocity_z: float):
        pass

    def check_if_mode_guided(self) -> bool:
        return True

    def connect(self):
        pass

    def get_current_location(self):
        pass

    def is_armed(self):
        return True

    def takeoff(self):
        pass

    def get_altitude(self):
        return 10 + random.random() * 5

    def get_heading(self):
        pass

    def follow_path(self, waypoints: List[Waypoint], source_obj: Source, detection_obj: ImageDetection):
        pass

    def get_initial_altitude(self):
        return 5

    def pid(self, target_position):
        pass

    def has_stopped(self):
        return False

    def stop_movement(self):
        pass

    def is_on_target(self, target_position: Tuple[int, int]):
        pass

    def distance_from_home(self) -> float:
        pass

    def return_to_launch(self):
        pass

    def land(self):
        pass

    def disconnect(self):
        pass

    def move_forward(self, distance):
        pass

    def log_and_print(self, message: str):
        pass

    def rotate(self, angle):
        pass

    def change_altitude(self, delta):
        pass

    def mission_completed(self):
        pass

    def assassinate(self):
        pass

    def get_vehicle_mode(self):
        return 'GUIDED'

    def get_battery_voltage(self):
        return 14.4