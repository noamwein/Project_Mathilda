import time

import numpy as np
from picamera2 import Picamera2

from BirdBrain.interfaces import Source


class PiCameraSource(Source):
    def __init__(self, resolution=(1280, 720), target_brightness=120, exposure_time=500, auto_gain_interval=0,
                 **kwargs):
        super().__init__(rotation=270, **kwargs)

        # Camera configuration
        self.resolution = resolution
        self.target_brightness = target_brightness
        self.picam2 = Picamera2()
        self.exposure_time = exposure_time
        self.max_gain = 50.0  # Maximum gain limit from camera control
        self.min_gain = 1.0  # Minimum gain limit from camera control

        # Auto-adjustment settings
        auto_gain_interval = auto_gain_interval  # Seconds
        self.last_gain_adjustment = time.time()

        # Configure camera
        self.configure_camera()

        self.auto_gain_interval = 0
        for _ in range(10):
            self.get_current_frame()
        self.auto_gain_interval = auto_gain_interval

    def configure_camera(self):
        """Set up the camera and configure it with the correct resolution."""
        # Configure the camera with the correct resolution and exposure
        config = self.picam2.create_still_configuration(main={"format": "RGB888", "size": self.resolution})
        self.picam2.configure(config)

        # Set initial exposure and gain
        self.picam2.set_controls({
            "AfMode": 2,
            "ExposureTime": self.exposure_time,
            "AeEnable": False,
            "AnalogueGain": 50.0  # Start with default gain
        })

        # Ensure the camera is ready
        time.sleep(0.2)

        # Start the camera
        self.picam2.start()
        time.sleep(2)  # Camera warm-up

        print('Exposure time configured:', self.exposure_time)

    def _get_current_frame(self):
        """Get the current frame, adjust gain if necessary, and return the image."""
        # Capture the current frame
        frame = self.picam2.capture_array()

        # Convert frame to brightness (average pixel intensity)
        current_brightness = np.mean(frame)

        # Adjust gain if necessary
        if time.time() - self.last_gain_adjustment >= self.auto_gain_interval:
            self.adjust_gain(current_brightness)
            self.last_gain_adjustment = time.time()

        return frame

    def adjust_gain(self, brightness):
        """Automatically adjust gain based on brightness."""
        metadata = self.picam2.capture_metadata()

        if "AnalogueGain" not in metadata:
            raise RuntimeError("Cannot find Lux or AnalogueGain in camera metadata!")

        gain = metadata["AnalogueGain"]

        print(f"[AutoGain] Brightness: {brightness:.2f} average pixel intensity, Gain: {gain:.2f}")

        if brightness < self.target_brightness / 1.2 and gain * 1.05 < self.max_gain:
            # Too dark -> increase gain
            new_gain = min(gain * 1.05, self.max_gain)
            print(f"[AutoGain] Increasing gain to {new_gain:.2f}")
            self.picam2.set_controls({"AnalogueGain": new_gain})

        elif brightness > self.target_brightness * 1.2 and gain / 1.05 > self.min_gain:
            # Too bright -> decrease gain
            new_gain = max(gain / 1.05, self.min_gain)
            print(f"[AutoGain] Decreasing gain to {new_gain:.2f}")
            self.picam2.set_controls({"AnalogueGain": new_gain})
