from picamera2 import Picamera2
import os
import cv2
from abc import ABC, abstractmethod

from BirdBrain.interfaces import Source


class PiCameraSource(Source):
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.start()
        print("PiCamera initialized and started.")

    def get_current_frame(self):
        try:
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame
        except Exception as e:
            print(f"An error occurred while capturing the frame: {e}")
            return None

    def stop_camera(self):
        self.picam2.stop()
        print("PiCamera stopped.")
