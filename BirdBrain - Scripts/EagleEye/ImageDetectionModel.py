from abc import ABC, abstractmethod
from Rogatka.interfaces import ImageDetection, Source
from EagleEye.ImageProcessingConstants import *
import EagleEye.FindBecker as FindBecker
import face_recognition
import cv2
import numpy as np

class ImageDetectionModel(ImageDetection):
    def __init__(self, reference_image_path: str, source: Source):
        self.reference_encoding = self.load_reference_encoding(reference_image_path)
        self.source = source
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

    def load_reference_encoding(self, reference_image_path):
        reference_image = face_recognition.load_image_file(reference_image_path)
        reference_encodings = face_recognition.face_encodings(reference_image)
        if not reference_encodings:
            raise ValueError("No face found in the reference image.")
        return reference_encodings[0]

    def detect_target(self, frame) -> bool:
        return bool(FindBecker.find_face_in_frame(frame, self.reference_encoding))

    def locate_target1(self, frame) -> tuple[int, int]:
        face_center = FindBecker.find_face_in_frame(frame, self.reference_encoding, scale=0.5,
                                                    tolerance=0.6)
        if not face_center:
            raise ValueError("Target not found in the frame.")
        return face_center

    def locate_target(self, frame):
        try:
            frame = self.source.get_current_frame()
        except ValueError:
            print("No more frames to process. Exiting...")
            return False

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_counter += 1

        if not self.face_found and self.frame_counter % PROCESS_EVERY_FRAMES == 0:
            try:
                x, y = self.locate_target1(frame)
                self.face_found = True
                self.bbox = (max(0, int(x - self.initial_bbox_size)), max(0, int(y - self.initial_bbox_size)),
                             min(frame.shape[1], int(x + self.initial_bbox_size)),
                             min(frame.shape[0], int(y + self.initial_bbox_size)))
                self.points = self.initialize_tracking_points(gray_frame, self.bbox)
                self.old_gray = gray_frame
            except ValueError:
                self.face_found = False
        elif self.face_found and self.points is not None:
            self.points = self.update_tracking_points(self.old_gray, gray_frame, self.points)
            filtered_points, avg_x, avg_y = self.filter_points(self.points, self.points)
            if filtered_points is not None and len(filtered_points) > self.min_tracking_points:
                self.bbox = (int(np.min(filtered_points[:, 0, 0])) - self.padding_pixels,
                             int(np.min(filtered_points[:, 0, 1])) - self.padding_pixels,
                             int(np.max(filtered_points[:, 0, 0])) + self.padding_pixels,
                             int(np.max(filtered_points[:, 0, 1])) + self.padding_pixels)
                self.draw_tracking(frame, filtered_points, avg_x, avg_y)
                self.position = self.export_position(int(avg_x), int(avg_y))
                self.draw_bounding_box(frame, self.bbox)
                self.points = self.add_more_interesting_points(gray_frame, self.bbox, filtered_points)
                self.points = self.points.reshape(-1, 1, 2)
                self.old_gray = gray_frame
            else:
                self.face_found = False

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return "over"

        return self.position

    def initialize_tracking_points(self, gray_frame, bbox):
        x_min, y_min, x_max, y_max = bbox
        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
        points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **self.feature_params)
        if points is not None:
            points[:, 0, 0] += x_min
            points[:, 0, 1] += y_min
        return points

    def update_tracking_points(self, old_gray, gray_frame, points):
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, points, None, **self.lk_params)
        return p1[st == 1] if p1 is not None else None

    def filter_points(self, new_points, points):
        if points is None or len(points) == 0:
            return None, None, None
        points = points.reshape(-1, 2)  # Reshape to (N, 2) for easier calculations
        avg_x = np.mean(points[:, 0])
        avg_y = np.mean(points[:, 1])
        distances = np.sqrt((points[:, 0] - avg_x) ** 2 + (points[:, 1] - avg_y) ** 2)
        if new_points is not None:
            min_length = min(len(new_points), len(distances))
            filtered_points = new_points[:min_length][distances[:min_length] <= DISTANCE_THRESHOLD]
        else:
            filtered_points = None
        return filtered_points.reshape(-1, 1, 2) if filtered_points is not None else None, avg_x, avg_y

    def draw_tracking(self, frame, points, avg_x, avg_y):
        for point in points:
            cv2.circle(frame, (int(point[0][0]), int(point[0][1])), 2, (0, 0, 255), -1)
        cv2.circle(frame, (int(avg_x), int(avg_y)), 25, (255, 0, 0), -1)

    def draw_bounding_box(self, frame, bbox):
        x_min, y_min, x_max, y_max = bbox
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    def add_more_interesting_points(self, gray_frame, bbox, points):
        x_min, y_min, x_max, y_max = bbox
        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
        new_points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **self.feature_params)
        if new_points is not None:
            new_points[:, 0, 0] += x_min
            new_points[:, 0, 1] += y_min
            points = np.vstack((points, new_points)) if points is not None else new_points
        return points

    def export_position(self, x, y):
        center_x = ORIGINAL_CAM_WIDTH // 2
        center_y = ORIGINAL_CAM_HEIGHT // 2
        return x - center_x, y - center_y

    def done(self):
        self.source.release()
        cv2.destroyAllWindows()
