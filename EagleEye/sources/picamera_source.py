import time
import numpy as np
from picamera2 import Picamera2

from BirdBrain.interfaces import Source

from BirdBrain.settings import (TARGET_BRIGHTNESS,
                                EXPOSURE_TIME, 
                                DEFAULT_GAIN,
                                LOW_RESOLUTION,
                                HIGH_RESOLUTION,
                                RESOLUTION)


class PiCameraSource(Source):
    def __init__(self, logging=False, target_brightness=TARGET_BRIGHTNESS, exposure_time=EXPOSURE_TIME,
                 default_gain=DEFAULT_GAIN, resolution=RESOLUTION, auto_gain_interval=0, **kwargs):
        super().__init__(rotation=270, **kwargs)

        # Camera configuration
        self.logging = logging
        self.target_brightness = target_brightness
        self.picam2 = Picamera2()

        self.exposure_time = exposure_time
        self.default_gain = default_gain
        self.resolution = resolution

        self.fps_list = []  # Used for calculating average fps

        # Auto-adjustment settings
        self.auto_gain_interval = auto_gain_interval  # Seconds
        self.last_gain_adjustment = time.time()

        # Configure camera
        self.configure_camera()

    def configure_camera(self):
        # [{'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (1280, 720), 'fps': 120.09,
        #   'crop_limits': (2064, 2032, 5120, 2880), 'exposure_limits': (69, 64497721, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (1920, 1080), 'fps': 60.04,
        #   'crop_limits': (784, 1312, 7680, 4320), 'exposure_limits': (107, 99943495, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (2312, 1736), 'fps': 30.0,
        #   'crop_limits': (0, 0, 9248, 6944), 'exposure_limits': (131, 122582952, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (3840, 2160), 'fps': 20.0,
        #   'crop_limits': (784, 1312, 7680, 4320), 'exposure_limits': (201, 187816992, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (4624, 3472), 'fps': 10.0,
        #   'crop_limits': (0, 0, 9248, 6944), 'exposure_limits': (254, 237625637, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (8000, 6000), 'fps': 3.0,
        #   'crop_limits': (624, 472, 9248, 6944), 'exposure_limits': (467, 435918849, None)},
        #  {'format': SRGGB10_CSI2P, 'unpacked': 'SRGGB10', 'bit_depth': 10, 'size': (9152, 6944), 'fps': 2.7,
        #   'crop_limits': (0, 0, 9248, 6944), 'exposure_limits': (467, 435918849, None)}]
        res = self.resolution

        frame_duration = int(1_000_000 / 200)  # Will be set to max FPS
        config = self.picam2.create_video_configuration(
            buffer_count=1, queue=False,
            main={"format": "RGB888", "size": res},
            controls={"FrameDurationLimits": (frame_duration, frame_duration)}
        )
        self.picam2.configure(config)

        # Set initial exposure and gain
        self.picam2.set_controls({
            "AfMode": 2,
            "ExposureTime": self.exposure_time,  # Will be set to minimum
            "AeEnable": False,
            "AnalogueGain": self.default_gain,
        })

        # Ensure the camera is ready
        time.sleep(0.2)
        self.picam2.start()

        time.sleep(2)  # Camera warm-up

        print('[Configuration]')
        metadata = self.picam2.capture_metadata()
        print("Exposure Time (micro seconds):", metadata.get("ExposureTime"))
        print("Frame Duration (micro seconds):", metadata.get("FrameDuration"))
        fps = 1_000_000 / metadata.get("FrameDuration")
        print(f"Estimated FPS: {fps:.2f}")
        print(self.picam2.camera_config)
        print()

    def _get_current_frame(self):
        """Get the current frame, adjust gain if necessary, and return the image."""
        # Capture the current frame
        before = time.time_ns()
        frame = self.picam2.capture_array()
        duration_ns = time.time_ns() - before
        fps = 1 / (duration_ns / 1e9)
        self.fps_list.append(fps)
        avg_fps = sum(self.fps_list) / len(self.fps_list)
        if self.logging:
            print('[FPS] Average FPS:', avg_fps, '\tCurrent FPS:', fps)

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
        if self.logging:
            print(f"[AutoGain] Brightness: {brightness:.2f} average pixel intensity, Gain: {gain:.2f}")

        if brightness < self.target_brightness / 1.2:
            # Too dark -> increase gain
            new_gain = gain * 1.1
            if self.logging:
                print(f"[AutoGain] Increasing gain to {new_gain:.2f}")
            self.picam2.set_controls({"AnalogueGain": new_gain})

        elif brightness > self.target_brightness * 1.2:
            # Too bright -> decrease gain
            new_gain = gain / 1.1
            if self.logging:
                print(f"[AutoGain] Decreasing gain to {new_gain:.2f}")
            self.picam2.set_controls({"AnalogueGain": new_gain})
