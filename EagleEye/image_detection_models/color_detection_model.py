import typing

import face_recognition
import numpy as np
import cv2

import EagleEye.FindBecker as FindBecker
from BirdBrain.interfaces import ImageDetection, Source
# from EagleEye.ImageProcessingConstants import *

# Constants
FRAME_WIDTH = 640  # Display frame width (resized window)
FRAME_HEIGHT = 480  # Display frame height (resized window)
CENTERED_X=0 #x's pixel of the dropped object
CENTERED_Y=0 #y's pixel of the dropped object

class ImageDetectionModel(ImageDetection):
    def __init__(self, reference_image_path: str, display: bool = True,
        always_recognize_person: bool = False):
        self.reference_image_path = reference_image_path
        self.display = display
        self.always_recognize_person = always_recognize_person
        self.bbox = None
        self.position = (0, 0)

    def detect_target(self, frame) -> bool:
        """
        Detects if there is any yellow area in the frame.
        """
        mask = self.extract_yellow(frame)
        yellow_pixels = cv2.countNonZero(mask)
        return yellow_pixels > 0

    def locate_target(self, frame):
        """
        Locates the largest yellow area and draws a bounding box around it.
        """
        mask = self.extract_yellow(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:  # Ignore small noise areas
                x, y, w, h = cv2.boundingRect(largest_contour)
                self.bbox = (x, x + w, y, y + h)
                self.position = self.export_position(x + w // 2, y + h // 2)
                self.draw_bounding_box(frame, self.bbox)
        else:
            self.position = (None, None)

        # Resize and display
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        if self.display:
            cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return "over"

        return self.position

    def draw_bounding_box(self, frame, bbox):
        x_min, x_max, y_min, y_max = bbox
        x_circle=(x_min+x_max)//2
        y_circle=(y_min+y_max)//2
        cv2.circle(frame, (x_circle,y_circle), 5, (0, 0, 255), -1)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)

    def export_position(self, x, y):
        # center_x = ORIGINAL_CAM_WIDTH // 2
        # center_y = ORIGINAL_CAM_HEIGHT // 2
        return x - CENTERED_X, y - CENTERED_Y

    def extract_yellow(self, frame):
        """
        Extracts yellow areas (like emergency vests) from the frame.
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for yellow color (very tolerant)
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Improve mask for blurred images
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)  # Smooth the mask

        return mask

def main():
    model = ImageDetectionModel(reference_image_path="", display=True)

    cap = cv2.VideoCapture(0)  # Open default webcam
    #print dimensions of the camera
    print("Camera dimensions:")
    print("Width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    CENTERED_X=cap.get(cv2.CAP_PROP_FRAME_WIDTH)//2
    CENTERED_Y=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)//2
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if model.detect_target(frame):
            pos = model.locate_target(frame)
            print(f"Target located at (relative): {pos}")
        else:
            print("No yellow detected.")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
