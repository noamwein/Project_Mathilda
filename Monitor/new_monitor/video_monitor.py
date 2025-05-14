from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QImage, QPixmap
import cv2

from BirdBrain.settings import (DROP_RADIUS,
                                YAW_TOLERANCE_THRESHOLD,
                                YAW_TOLERANCE_RADIUS,
                                CENTERED_X, CENTERED_Y)


class VideoMonitor(MonitorPanel):
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
        h, w, ch = processed_frame.shape
        bytes_per_line = ch * w
        qt_img = QImage(processed_frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
        self.label.setPixmap(QPixmap.fromImage(qt_img))

    def _draw_overlay(self, frame):
        processed_frame = frame.copy()
        self._draw_shapes(processed_frame)
        # bbox = self.image_detection.image_detection_data.get('bbox')
        # if bbox:
        #     self._draw_bounding_box(processed_frame, bbox)
        return processed_frame

    def _draw_shapes(self, frame):
        cross_color = (0, 0, 255)  # Red
        drop_color = (17, 250, 231)  # Yellow
        yaw_color = (219, 204, 101)  # Light Blue

        center_x, center_y = self.get_center_pos()
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

    # def _draw_bounding_box(self, frame, bbox):
    #     x_min, x_max, y_min, y_max = bbox
    #     x_circle = (x_min + x_max) // 2
    #     y_circle = (y_min + y_max) // 2
    #     cv2.circle(frame, (x_circle, y_circle), 5, (255, 0, 0), -1)
    #     cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)
