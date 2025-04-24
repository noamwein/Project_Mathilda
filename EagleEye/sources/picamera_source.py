import logging
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from picamera2 import Picamera2, Preview
import os
import cv2
from abc import ABC, abstractmethod

from BirdBrain.interfaces import Source


class PiCameraSource(Source):
    def __init__(self):
        self.picam2 = Picamera2()

        # Configure camera with the lowest exposure time and infinite focus
        config = self.picam2.create_still_configuration(main={"format": "RGB888", "size": (640, 480)})
        self.picam2.configure(config)

        # Apply camera controls
        self.picam2.set_controls({
            "AfMode": 2,                      # Continuous auto-focus
            # "LensPosition": 5.8,              # Infinite focus (if supported)
            "AeEnable": False,                # Disable auto-exposure
            "ExposureTime": 1000,              # Minimum exposure in microseconds (may vary per sensor)
            "AnalogueGain": 20.0               # Minimum analog gain
        })

        self.picam2.start()
        print("PiCamera initialized and started.")

    def get_current_frame(self):
        try:
            frame = self.picam2.capture_array()
            return frame
        except Exception as e:
            print(f"An error occurred while capturing the frame: {e}")
            return None

    def stop_camera(self):
        self.picam2.stop()
        print("PiCamera stopped.")
