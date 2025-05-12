import typing

import face_recognition
import numpy as np

import EagleEye.FindBecker as FindBecker
from BirdBrain.interfaces import ImageDetection, Source
from EagleEye.ImageProcessingConstants import *

from BirdBrain.settings import (MAX_BBOX_HEIGHT,
                                MAX_BBOX_WIDTH,
                                DELAY_IN_DETECTION)

class ImageDetectionModel(ImageDetection):
    def __init__(self, reference_image_path: str, display: bool = True,
        always_recognize_person: bool = False):
        self.reference_encoding = self.load_reference_encoding(reference_image_path)
        self.face_found = False
        self.frame_counter = 0
        self.old_gray = None
        self.points = None
        self.bbox = None
        self.feature_params = FEATURE_PARAMS
        self.lk_params = LK_PARAMS
        self.min_tracking_points = MIN_TRACKING_POINTS
        self.padding_pixels = PADDING_PIXELS
        self.distance_threshold = DISTANCE_THRESHOLD
        self.initial_bbox_size = INITIAL_BBOX_SIZE
        self.position = None, None
        self.display = display
        self.always_recognize_person = always_recognize_person
        self.delay_in_detection = DELAY_IN_DETECTION
        self.og_face_size = None

    def load_reference_encoding(self, reference_image_path):
        reference_image = face_recognition.load_image_file(reference_image_path)
        reference_encodings = face_recognition.face_encodings(reference_image)
        if not reference_encodings:
            raise ValueError("No face found in the reference image.")
        return reference_encodings[0]

    def detect_target(self, frame) -> bool:
        return bool(FindBecker.find_face_in_frame(frame, self.reference_encoding))

    def recognize_person(self, frame) -> typing.Tuple[int, int, int, int]:
        face_coordinates = FindBecker.find_face_in_frame(frame, self.reference_encoding, scale=0.5,
                                                    tolerance=0.6)
        if not face_coordinates:
            raise ValueError("Target not found in the frame.")
        return face_coordinates

    def locate_target(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_counter += 1

        if (self.always_recognize_person or not self.face_found) and self.frame_counter % self.delay_in_detection == 0:
            self.delay_in_detection = DELAY_IN_DETECTION
            try:
                face_bounds = self.recognize_person(frame)
                self.face_found = True
                self.bbox = tuple(map(int, face_bounds))  # (x_min, x_max, y_min, y_max)
                self.og_face_size = (self.bbox[1] - self.bbox[0], self.bbox[3] - self.bbox[2])
                self.points = self.initialize_tracking_points(gray_frame, self.bbox)
                self.old_gray = gray_frame
            except ValueError:
                print("No face found in the frame.")
                self.face_found = False

        elif (self.always_recognize_person or not self.face_found) and self.frame_counter % self.delay_in_detection != 0:
            self.delay_in_detection -= 1

        elif self.face_found and self.points is not None:
            # Update tracking points
            self.points = self.update_tracking_points(self.old_gray, gray_frame, self.points)

            # Filter points
            filtered_points, avg_x, avg_y = self.filter_points(self.points, self.points)

            if filtered_points is not None and len(filtered_points) > self.min_tracking_points:
                # Efficient bounding box calculation with NumPy
                bbox_array = filtered_points[:, 0]
                self.bbox = (
                    int(np.min(bbox_array[:, 0])),  # x_min
                    int(np.max(bbox_array[:, 0])),  # x_max
                    int(np.min(bbox_array[:, 1])),  # y_min
                    int(np.max(bbox_array[:, 1])),  # y_max
                )

                # Draw tracking and bounding box
                self.draw_tracking(frame, filtered_points, avg_x, avg_y)
                self.draw_bounding_box(frame, self.bbox)

                # Add more points if needed
                if len(filtered_points) < 20:
                    self.points = self.add_more_interesting_points(gray_frame, self.bbox, filtered_points)
                    self.points, avg_x, avg_y = self.filter_points(self.points, self.points)
                self.points = self.points.reshape(-1, 1, 2)

            else:
                # Add new points if tracking fails
                self.points = self.add_more_interesting_points(gray_frame, self.bbox, None)
                self.points, avg_x, avg_y = self.filter_points(self.points, self.points)
                if self.points is None or len(self.points) < self.min_tracking_points:
                    self.face_found = False

            # Validate bounding box size
            bbox_width = self.bbox[1] - self.bbox[0]
            bbox_height = self.bbox[3] - self.bbox[2]

            if bbox_width >  (5 * self.og_face_size[0]) and bbox_height > (5 * self.og_face_size[1]):
                print("Face lost due to excessive bounding box size.")
                self.face_found = False

            # If points are empty, reset detection
            if self.points is None or len(self.points) == 0:
                self.face_found = False
                self.delay_in_detection = DELAY_IN_DETECTION
                return None, None

            self.position = self.export_position(avg_x, avg_y)

            # Update old frame for optical flow
            self.old_gray = gray_frame

        # Resize and display frame
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        if self.display:
            cv2.imshow('Video', frame)

        # Check for exit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return "over"

        return self.position

    def initialize_tracking_points(self, gray_frame, bbox):
        x_min, x_max, y_min, y_max = bbox
        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
        points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **self.feature_params)
        if points is not None:
            points[:, 0, 0] += x_min
            points[:, 0, 1] += y_min
        return points if points is not None and len(points) > 0 else None

    def update_tracking_points(self, old_gray, gray_frame, points):
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, points, None, **self.lk_params)
        good_new = p1[st == 1] if p1 is not None else None
        return good_new

    def filter_points(self, new_points, points):
        if points is None or len(points) == 0:
            return None, None, None
        points = points.reshape(-1, 2)  # Reshape to (N, 2)
        avg_x = np.median(points[:, 0])
        avg_y = np.median(points[:, 1])
        distances = np.sqrt((points[:, 0] - avg_x) ** 2 + (points[:, 1] - avg_y) ** 2)
        if new_points is not None and len(new_points) > 0:
            new_points = new_points.reshape(-1, 2)
            bbox_width = np.max(points[:, 0]) - np.min(points[:, 0])
            bbox_height = np.max(points[:, 1]) - np.min(points[:, 1])
            threshold = min(bbox_width, bbox_height)
            
            # Filter points based on distance threshold
            valid_indices = distances <= threshold
            filtered_points = points[valid_indices] if np.any(valid_indices) else None
            if filtered_points is not None:
                return filtered_points.reshape(-1, 1, 2) if len(filtered_points) > 0 else None, avg_x, avg_y
        return None, avg_x, avg_y

    def draw_tracking(self, frame, points, avg_x, avg_y):
        for point in points:
            cv2.circle(frame, (int(point[0][0]), int(point[0][1])), 5, (0, 0, 255), -1)
        cv2.circle(frame, (int(avg_x), int(avg_y)), 25, (255, 0, 0), -1)

    def draw_bounding_box(self, frame, bbox):
        x_min, x_max, y_min, y_max = bbox
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

    def add_more_interesting_points(self, gray_frame, bbox, points):
        x_min, x_max, y_min, y_max = bbox
        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
        frame_height, frame_width = gray_frame.shape[:2]
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min

        # Calculate the ratio of bbox size to frame size
        width_ratio = bbox_width / frame_width
        height_ratio = bbox_height / frame_height

        # Determine buffer size based on the ratio (smaller ratio -> larger buffer)
        buffer_factor = max(1.0, 1.0 / (20 * (width_ratio + height_ratio)))
        bbox_width = min(bbox_width, int(2.5 * self.og_face_size[0]))
        bbox_height = min(bbox_height, int(2.5 * self.og_face_size[1]))
        buffer_x = int(bbox_width * buffer_factor)
        buffer_y = int(bbox_height * buffer_factor)

        # Apply buffer to the region of interest
        x_min = max(0, x_min - buffer_x)
        x_max = min(frame_width, x_max + buffer_x)
        y_min = max(0, y_min - buffer_y)
        y_max = min(frame_height, y_max + buffer_y)

        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
        new_points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **self.feature_params)
        if new_points is not None:
            new_points[:, 0, 0] += x_min
            new_points[:, 0, 1] += y_min
            points = np.vstack((points, new_points)) if points is not None else new_points
        return points

    def export_position(self, x, y):
        # center_x = ORIGINAL_CAM_WIDTH // 2
        # center_y = ORIGINAL_CAM_HEIGHT // 2
        return x - CENTERED_X, y - CENTERED_Y
