import math
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
from Monitor.video_saver import MP4VideoSaver
from Rogatka.dummy_servo import DummyServo
from BirdBrain.interfaces import GUI, Servo
from BirdBrain.settings import DROP_RADIUS, YAW_TOLERANCE_THRESHOLD, YAW_TOLERANCE_RADIUS
from Rogatka.drone_client import State

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


def resize_and_pad(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    """
    Resize an image to fit into a target frame while maintaining aspect ratio,
    and pad with black pixels to match the exact target size.

    Parameters:
        frame (np.ndarray): The input image.
        target_width (int): Target frame width.
        target_height (int): Target frame height.

    Returns:
        np.ndarray: The resized and padded image.
    """
    original_height, original_width = frame.shape[:2]

    # Compute the scaling factor to fit the image into the target frame
    scale = min(target_width / original_width, target_height / original_height)

    # Resize image while maintaining aspect ratio
    new_width = int(original_width * scale)
    new_height = int(original_height * scale)
    resized_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)

    # Create a black canvas of target size
    padded_frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Compute top-left corner to place the resized image
    x_offset = (target_width - new_width) // 2
    y_offset = (target_height - new_height) // 2

    # Place the resized image onto the canvas
    padded_frame[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized_frame

    return padded_frame


class MonitorGUI(GUI):
    def __init__(self, drone_client: DroneClient, video_saver: VideoSaver, image_detection: ImageDetection,
                 servo: Servo = None,
                 enable_display=True):
        super().__init__(drone_client=drone_client, video_saver=video_saver, image_detection=image_detection,
                         servo=servo,
                         enable_display=enable_display)

        # Use tkinter to get screen resolution
        root = tk.Tk()
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy()
        margin = 50

        self.frame_dims = (screen_width // 2 - margin, screen_height - 2 * margin)

        # Create a named window
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
        # Resize to screen size minus margin
        cv2.resizeWindow("Video", self.frame_dims[0], self.frame_dims[1])

        # Move to top-left corner
        cv2.moveWindow("Video", screen_width // 2, margin)

        self.prev_net = 0, 0

    def draw_gui(self, frame):
        if frame is None:
            return
        processed_frame = self._draw_gui(frame)
        if self.video_saver is not None:
            self.video_saver.write_frame(processed_frame)
        if self.enable_display:
            cv2.imshow('Video', processed_frame)
            pass

        if cv2.waitKey(1) == CLOSE_WINDOW_KEY:
            self.enable_display = False
            self.close()

    def _draw_gui(self, frame):
        processed_frame = frame.copy()
        self.draw_shapes(processed_frame)
        bbox = self.image_detection.image_detection_data.get('bbox')
        if bbox:
            self.draw_bounding_box(processed_frame, bbox)
        self.draw_bombs(processed_frame)
        processed_frame = self.get_monitor(processed_frame)
        processed_frame = resize_and_pad(processed_frame, target_height=self.frame_dims[1],
                                         target_width=self.frame_dims[0])
        return processed_frame

    def draw_shapes(self, frame):
        cross_color = (0, 0, 255)  # Red
        drop_color = (17, 250, 231)  # Yellow
        yaw_color = (219, 204, 101)  # Light Blue
        # Draw cross
        cv2.line(frame, (CENTERED_X - 80, CENTERED_Y), (CENTERED_X + 80, CENTERED_Y), cross_color, 6)
        cv2.line(frame, (CENTERED_X, CENTERED_Y - 80), (CENTERED_X, CENTERED_Y + 80), cross_color, 6)
        # Draw circles
        cv2.circle(frame, (CENTERED_X, CENTERED_Y), DROP_RADIUS, drop_color, 6)
        cv2.circle(frame, (CENTERED_X, CENTERED_Y), YAW_TOLERANCE_RADIUS, yaw_color, 6)
        # Draw yaw pixel threshold
        cv2.line(frame, (CENTERED_X + YAW_TOLERANCE_THRESHOLD, 0),
                 (CENTERED_X + YAW_TOLERANCE_THRESHOLD, frame.shape[0]), yaw_color, 6)
        cv2.line(frame, (CENTERED_X - YAW_TOLERANCE_THRESHOLD, 0),
                 (CENTERED_X - YAW_TOLERANCE_THRESHOLD, frame.shape[0]), yaw_color, 6)

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
            f'CPU TEMP: {get_cpu_temp():.2f} deg',
            f'UPLOAD:   {upload_speed:.2f} KB/s',
            f'DOWNLOAD: {download_speed:.2f} KB/s',
            # TODO: number of remaining bombs
            # TODO: pi command sent to pixhawk
        ])
        return self.add_side_panel(frame, monitor_text)

    def add_side_panel(self, frame, text, font_size=50, padding=20):
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
            font = ImageFont.load_default(size=font_size)

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
        y = padding
        for i, line in enumerate(lines):
            draw.text(
                (frame.shape[1] + padding, y),
                line,
                font=font,
                fill=(0, 0, 0)
            )
            y += line_heights[i] + 2 * padding

        return cv2.cvtColor(np.array(full_img), cv2.COLOR_RGB2BGR)

    def draw_bombs(self, frame):
        """
        Draws bomb images in the bottom-right corner of the frame.
        The number of bombs is determined by self.servo.get_bombs_left() (0 to 3).
        """

        bombs_left = self.servo.get_bombs_left()
        if bombs_left <= 0:
            return

        # Path to bomb icon image
        bomb_image_path = r"assets\bomb.png"
        icon_size = 40  # Width/height of each bomb icon
        margin = 10  # Space between icons and edges

        # Load and resize the bomb image
        if not os.path.exists(bomb_image_path):
            print(f"Bomb image not found: {bomb_image_path}")
            return

        bomb_img = Image.open(bomb_image_path).convert("RGBA").resize((icon_size, icon_size))

        # Convert OpenCV frame to RGBA PIL Image
        pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")

        # Image size
        frame_width, frame_height = pil_img.size
        # Start from the bottom-right corner and move left
        for i in range(bombs_left):
            x = frame_width - margin - (icon_size * (i + 1)) - (margin * i)
            y = frame_height - icon_size - margin
            pil_img.paste(bomb_img, (x, y), bomb_img)  # Paste with alpha mask

        # Convert back to OpenCV format
        frame[:] = cv2.cvtColor(np.array(pil_img.convert("RGB")), cv2.COLOR_RGB2BGR)

    def get_direction(self):
        mode = self.drone_client.get_mode()
        rotation_direction = 'clockwise'  # Or dynamically change this too
        vx = 0
        vy = 0
        return mode, rotation_direction, vx, vy

    def draw_drone_illus(self, frame):
        """
        Draws a drone illustration in the top-right corner of the frame.
        - In movement mode: displays a rotated arrow based on vx, vy.
        - In rotation mode: shows a clockwise or counterclockwise arc with a blue circle.
        """
        mode, rotation_direction, vx, vy = self.get_direction()

        h, w = frame.shape[:2]
        overlay_pos = (w - 60, 20)  # Position in the top-right corner of the frame

        if mode == State.MOVEMENT:
            # Calculate angle and stretch arrow based on velocity (vx, vy)
            angle = -math.degrees(math.atan2(vy, vx))  # Get angle from vx, vy
            norm = math.hypot(vx, vy)
            if norm == 0:
                norm = 0.001
            arrow_length = int(norm * 25)  # You can adjust the scaling factor here

            # Draw arrow
            center = (overlay_pos[0] + 30, overlay_pos[1] + 30)
            cv2.arrowedLine(frame, center,
                            (center[0] + int(arrow_length * vx / norm),
                             center[1] - int(arrow_length * vy / norm)),
                            (0, 255, 0), 3)  # Green arrow

        elif mode == State.ROTATION:
            # For rotation, draw a quarter-circle arc and a blue circle at the head
            center = (overlay_pos[0] + 30, overlay_pos[1] + 30)
            radius = 20

            if rotation_direction == 'clockwise':
                start_angle = 0
                end_angle = -90
                angle_rad = math.radians(0)

            else:  # Counterclockwise
                start_angle = -90
                end_angle = -180
                angle_rad = math.radians(180)

            color = (0, 255, 0)  # Green for clockwise
            # Draw the arc (quarter circle)
            cv2.ellipse(frame, center, (radius, radius), 0, start_angle, end_angle, color, 3)

            # Draw blue circle at the end of the arc (head of the arrow)
            tip_x = int(center[0] + radius * math.cos(angle_rad))
            tip_y = int(center[1] + radius * math.sin(angle_rad))
            cv2.circle(frame, (tip_x, tip_y), 5, (255, 0, 0), -1)  # Blue circle

        # Optionally, you can add text for debugging purposes
        if mode == 'movement' and norm > 1e-3:
            cv2.putText(frame, 'Movement', (overlay_pos[0] - 20, overlay_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        elif mode == 'rotation':
            cv2.putText(frame, 'Rotation', (overlay_pos[0] - 20, overlay_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.putText(frame, 'Stopped', (overlay_pos[0] - 20, overlay_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def close(self):
        self.video_saver.save_and_close()
        # destroy all OpenCV windows
        cv2.destroyAllWindows()
        self.video_saver = None
        self.enable_display = False


def main():
    source = CameraSource()
    # gui = MonitorGUI(drone_client=DummyClient(), video_saver=PiVideoSaver(),
    #                  image_detection=ColorImageDetectionModel(None), servo=ServoMotor())
    gui = MonitorGUI(drone_client=DummyClient(), video_saver=MP4VideoSaver(),
                     image_detection=ColorImageDetectionModel(None), servo=DummyServo())
    for _ in range(1000):
        frame = source.get_current_frame()
        gui.draw_gui(frame)
    gui.close()


if __name__ == '__main__':
    main()
