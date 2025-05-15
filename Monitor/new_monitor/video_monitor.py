import math
from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QImage, QPixmap
import cv2
from Monitor.new_monitor.utils import _resize_and_pad

from BirdBrain.settings import (DROP_RADIUS,
                                YAW_TOLERANCE_THRESHOLD,
                                YAW_TOLERANCE_RADIUS,
                                CENTERED_X, CENTERED_Y)
from BirdBrain.interfaces import ImageDetection, DroneClient


class VideoMonitor(MonitorPanel):
    def __init__(self, parent=None, image_detection : ImageDetection = None, drone_client: DroneClient = None):
        super().__init__(parent=parent)
        self.image_detection = image_detection
        self.drone_client = drone_client


    def setup_ui(self):
        self.label = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        frame = data.cv2_frame  # numpy array
        if frame is None:
            return
        processed_frame = self._draw_overlay(frame)
        final_frame = _resize_and_pad(processed_frame, self.width(), self.height())
        h, w, ch = final_frame.shape
        bytes_per_line = ch * w
        qt_img = QImage(final_frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
        self.label.setPixmap(QPixmap.fromImage(qt_img))

    def _draw_overlay(self, frame):
        processed_frame = frame.copy()
        self._draw_shapes(processed_frame)
        bbox = self.image_detection.image_detection_data.get('bbox')
        if bbox:
            self._draw_bounding_box(processed_frame, bbox)
        return processed_frame

    def _draw_shapes(self, frame):
        cross_color = (0, 0, 255)  # Red
        drop_color = (17, 250, 231)  # Yellow
        yaw_color = (219, 204, 101)  # Light Blue

        center_x, center_y = self._get_center_pos()
        # Draw cross
        cv2.line(frame, (center_x - 80, center_y), (center_x + 80, center_y), cross_color, 6)
        cv2.line(frame, (center_x, center_y - 80), (center_x, center_y + 80), cross_color, 6)
        # Draw circles
        cv2.circle(frame, (center_x, center_y), DROP_RADIUS, drop_color, 6)
        cv2.circle(frame, (center_x, center_y), YAW_TOLERANCE_RADIUS, yaw_color, 6)
        # Draw yaw pixel threshold
        cv2.line(frame, (center_x + YAW_TOLERANCE_THRESHOLD, 0),
                 (center_x + YAW_TOLERANCE_THRESHOLD, frame.shape[0]), yaw_color, 6)
        cv2.line(frame, (center_x - YAW_TOLERANCE_THRESHOLD, 0),
                 (center_x - YAW_TOLERANCE_THRESHOLD, frame.shape[0]), yaw_color, 6)
        self.draw_drone_illus(frame)


    def _draw_bounding_box(self, frame, bbox):
        x_min, x_max, y_min, y_max = bbox
        x_circle = (x_min + x_max) // 2
        y_circle = (y_min + y_max) // 2
        cv2.circle(frame, (x_circle, y_circle), 5, (255, 0, 0), -1)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

    def _get_center_pos(self):
        try:
            return self.drone_client.get_center_position()
        except:
            return CENTERED_X, CENTERED_Y

    def draw_drone_illus(self, frame):
        """
        Draws a drone illustration in the top-right corner of the frame.
        - In movement mode: displays a rotated arrow based on vx, vy.
        - In rotation mode: shows a clockwise or counterclockwise arc with a blue circle.
        """
        try:
            yaw=math.degrees(self.drone_client.get_yaw())
        except Exception:
            yaw=0
        ARROW_LENGTH = 60  # pixels
        COLOR = (0, 255, 0)  # Green arrow
        THICKNESS = 2
        
        # Compute center of the arrow illustration
        h, w, _ = frame.shape
        center = (w - 70, 70)  # top-right corner with some padding

        # Convert yaw to radians and rotate counter-clockwise (OpenCV uses standard math coords)
        angle_rad = math.radians(-yaw + 90)  # +90 to make 0 deg point up

        # Calculate arrow endpoint
        end_x = int(center[0] + ARROW_LENGTH * math.cos(angle_rad))
        end_y = int(center[1] - ARROW_LENGTH * math.sin(angle_rad))

        # Draw arrow
        cv2.arrowedLine(frame, center, (end_x, end_y), COLOR, THICKNESS, tipLength=0.3)

        # Optional: draw a small circle at center
        cv2.circle(frame, center, 3, (255, 0, 0), -1)
