import subprocess
import tkinter as tk

import cv2
import psutil
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from EagleEye.image_detection_models.color_detection_model import CENTERED_X, CENTERED_Y, ImageDetection
from EagleEye.image_detection_models.color_detection_model import ColorImageDetectionModel
from EagleEye.sources.camera_source import CameraSource
from Monitor.video_saver import VideoSaver
from Rogatka.drone_client import DroneClient
from Rogatka.dummy_client import DummyClient

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


class GUI:
    def __init__(self, drone_client: DroneClient, video_saver: VideoSaver, image_detection: ImageDetection,
                 enable_display=True):
        self.drone_client = drone_client
        self.video_saver = video_saver
        self.enable_display = enable_display
        self.image_detection = image_detection

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
        self.draw_monitor(processed_frame)
        return processed_frame

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

    def draw_monitor(self, frame):
        net_now, upload_speed, download_speed = get_bandwidth(self.prev_net)
        self.prev_net = net_now
        monitor_text = '\n'.join([
            f'altitude: {self.drone_client.get_altitude():.2f} m',
            f'mode: {self.drone_client.get_vehicle_mode()}',
            f'battery: {self.drone_client.get_battery_voltage():.2f} V',
            '',
            f'cpu temp: {get_cpu_temp():.2f} deg',
            f'upload: {upload_speed:.2f} KB/s',
            f'download: {download_speed:.2f} KB/s',
            # TODO: number of remaining bombs
            # TODO: pi command sent to pixhawk
        ])
        self.draw_text(frame, monitor_text, None, None, 0, 0)

    def draw_text(self, frame, text, width=None, height=None, margin_x=20, margin_y=20,
                  font_path="consola.ttf", font_size=20, padding=10):
        """
        Draw a textbox at bottom-right with optional auto-sizing and multiline support using a custom font.
        """

        # Convert frame to Pillow image
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(frame_rgb)
        draw = ImageDraw.Draw(pil_img)

        # Load custom font
        try:
            font = ImageFont.truetype(font_path, font_size)
        except IOError:
            raise ValueError(f"Font not found at {font_path}")

        # Measure each line
        lines = text.split('\n')
        line_sizes = [draw.textbbox((0, 0), line, font=font) for line in lines]
        line_heights = [bbox[3] - bbox[1] for bbox in line_sizes]
        line_widths = [bbox[2] - bbox[0] for bbox in line_sizes]

        text_width = max(line_widths)
        text_height = sum(line_heights) + 5 * (len(lines) - 1)

        # Determine box size if not provided
        box_width = width if width is not None else text_width + 2 * padding
        box_height = height if height is not None else text_height + 2 * padding

        # Calculate box position (bottom-right)
        img_w, img_h = frame.shape[1], frame.shape[0]
        x2, y2 = img_w - margin_x, img_h - margin_y
        x1, y1 = x2 - box_width, y2 - box_height

        # Draw background box
        draw.rectangle([x1, y1, x2, y2], fill=(0, 0, 0, 255))

        # Draw each line centered
        y_cursor = y1 + padding
        for i, line in enumerate(lines):
            line_width = line_widths[i]
            line_height = line_heights[i]
            x_text = x1 + (box_width - line_width) // 2
            draw.text((x_text, y_cursor), line, font=font, fill=(255, 255, 255))
            y_cursor += line_height + 5

        # Convert back to OpenCV image
        frame[:] = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    def close(self):
        cv2.destroyAllWindows()


def main():
    source = CameraSource()
    gui = GUI(drone_client=DummyClient(), video_saver=VideoSaver(), image_detection=ColorImageDetectionModel(None))
    while True:
        frame = source.get_current_frame()
        gui.draw_gui(frame)


if __name__ == '__main__':
    main()
