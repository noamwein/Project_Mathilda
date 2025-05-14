from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QImage, QPixmap
import cv2
import numpy as np

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

def _resize_and_pad(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
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

