import os
import subprocess
import sys
import tkinter as tk

import cv2
import numpy as np
import psutil
from PIL import Image, ImageDraw, ImageFont

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from EagleEye.image_detection_models.color_detection_model import CENTERED_X, CENTERED_Y, ImageDetection
from EagleEye.image_detection_models.color_detection_model import ColorImageDetectionModel
from EagleEye.sources.camera_source import CameraSource
from Monitor.video_saver import VideoSaver
from Rogatka.drone_client import DroneClient
from Rogatka.dummy_client import DummyClient
from Monitor.video_saver import PiVideoSaver
from BirdBrain.interfaces import GUI

CLOSE_WINDOW_KEY = 27  # escape


def get_cpu_temp():
    try:
        output = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        temp_str = output.strip().replace("temp=", "").replace("'C", "")
        return float(temp_str)
    except Exception:
        return 0.0


def get_bandwidth(prev):
    counters = psutil.net_io_counters()
    upload = counters.bytes_sent
    download = counters.bytes_recv
    up_speed = (upload - prev[0]) / 1024.0  # KB/s
    down_speed = (download - prev[1]) / 1024.0
    return (upload, download), up_speed, down_speed


class MonitorGUI(GUI):
    def __init__(self, drone_client: DroneClient, video_saver: VideoSaver, image_detection: ImageDetection,
                 enable_display=True):
        super().__init__(drone_client=drone_client, video_saver=video_saver, image_detection=image_detection,
                         enable_display=enable_display)

        # Use tkinter to get screen resolution
        root = tk.Tk()
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy()
        margin = 50

        # Create a named window
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
        # Resize to screen size minus margin
        cv2.resizeWindow("Video", screen_width // 2 - margin, screen_height - margin * 2)

        # Move to top-left corner
        cv2.moveWindow("Video", screen_width // 2 - margin, margin)

        self.prev_net = 0, 0

    def draw_gui(self, frame):
        if frame is None:
            return
        processed_frame = self._draw_gui(frame)
        if self.video_saver is not None:
            self.video_saver.write_frame(processed_frame)
        if self.enable_display:
            cv2.imshow('Video', processed_frame)

        if cv2.waitKey(1) == CLOSE_WINDOW_KEY:
            self.enable_display = False
            self.close()

    def _draw_gui(self, frame):
        bbox = self.image_detection.image_detection_data.get('bbox')
        processed_frame = frame.copy()
        self.draw_cross(processed_frame)
        if bbox:
            self.draw_bounding_box(frame, bbox)
        monitor = self.get_monitor(processed_frame)
        return monitor

    def draw_cross(self, frame):
        """
        Draws a cross at the center of the frame.
        """
        cv2.line(frame, (CENTERED_X - 80, CENTERED_Y), (CENTERED_X + 80, CENTERED_Y), (0, 0, 255), 6)
        cv2.line(frame, (CENTERED_X, CENTERED_Y - 80), (CENTERED_X, CENTERED_Y + 80), (0, 0, 255), 6)

    def draw_bounding_box(self, frame, bbox):
        x_min, x_max, y_min, y_max = bbox
        x_circle = (x_min + x_max) // 2
        y_circle = (y_min + y_max) // 2
        cv2.circle(frame, (x_circle, y_circle), 5, (255, 0, 0), -1)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

    def get_monitor(self, frame):
        net_now, upload_speed, download_speed = get_bandwidth(self.prev_net)
        self.prev_net = net_now
        battery_voltage = vehicle_mode = altitude = 'unknown'

        try:
            altitude = f'{self.drone_client.get_altitude():.2f} m'
        except Exception:
            pass

        try:
            vehicle_mode = str(self.drone_client.get_vehicle_mode()).split(':')[1]
        except Exception:
            pass

        try:
            battery_voltage = f'{self.drone_client.get_battery_voltage():.2f} V'
        except Exception:
            pass

        monitor_text = '\n'.join([
            f'ALTITUDE: {altitude}',
            f'MODE:     {vehicle_mode}',
            f'BATTERY:  {battery_voltage}',
            '',
            f'CPU TEMP: {get_cpu_temp():.2f} deg',
            f'UPLOAD:   {upload_speed:.2f} KB/s',
            f'DOWNLOAD: {download_speed:.2f} KB/s',
            # TODO: number of remaining bombs
            # TODO: pi command sent to pixhawk
        ])
        return self.add_side_panel(frame, monitor_text)

    def add_side_panel(self, frame, text, font_size=40, padding=10, margin_y=20):
        """
        Adds a side panel to the right of the frame and draws text on it.
        Returns the new extended frame.
        """

        # Convert to Pillow image to measure text
        pil_img = Image.new("RGB", (1, 1))
        draw = ImageDraw.Draw(pil_img)

        # Use default monospaced system font
        fallback_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
            "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
        ]
        font = None
        for path in fallback_paths:
            if os.path.exists(path):
                try:
                    font = ImageFont.truetype(path, font_size)
                    break
                except:
                    pass
        if font is None:
            font = ImageFont.load_default()

        # Measure text
        lines = text.split('\n')
        line_sizes = [draw.textbbox((0, 0), line, font=font) for line in lines]
        line_widths = [bbox[2] - bbox[0] for bbox in line_sizes]
        line_heights = [bbox[3] - bbox[1] for bbox in line_sizes]

        panel_width = max(line_widths) + 2 * padding

        # Create new extended frame
        new_width = frame.shape[1] + panel_width
        new_frame = np.zeros((frame.shape[0], new_width, 3), dtype=np.uint8)
        new_frame[:, :frame.shape[1]] = frame  # copy original frame
        new_frame[:, frame.shape[1]:] = (255, 255, 255)  # fill side panel

        # Convert to Pillow for text drawing
        full_img = Image.fromarray(cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(full_img)

        # Draw text in panel
        y = margin_y
        for i, line in enumerate(lines):
            draw.text(
                (frame.shape[1] + padding, y),
                line,
                font=font,
                fill=(0, 0, 0)
            )
            y += line_heights[i] + 5

        return cv2.cvtColor(np.array(full_img), cv2.COLOR_RGB2BGR)

    def close(self):
        self.video_saver.save_and_close()
        # destroy all OpenCV windows
        cv2.destroyAllWindows()


def main():
    source = CameraSource()
    gui = MonitorGUI(drone_client=DummyClient(), video_saver=PiVideoSaver(),
                     image_detection=ColorImageDetectionModel(None))
    while True:
        frame = source.get_current_frame()
        gui.draw_gui(frame)


if __name__ == '__main__':
    main()
