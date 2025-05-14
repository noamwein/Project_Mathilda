import cv2
import numpy as np

from BirdBrain.interfaces import ImageDetection

# from EagleEye.ImageProcessingConstants import *

from BirdBrain.settings import (FRAME_WIDTH,
                                FRAME_HEIGHT,
                                CENTERED_X,
                                CENTERED_Y)


class ColorImageDetectionModel(ImageDetection):
    def __init__(self, reference_image_path: str, always_recognize_person: bool = False):
        super().__init__()
        self.reference_image_path = reference_image_path
        self.always_recognize_person = always_recognize_person
        self.bbox = None
        self.position = (0, 0)

    def detect_target(self, frame) -> bool:
        """
        Detects if there is any yellow area in the frame.
        """
        return self.locate_target(frame) != (None, None)

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
                self.position = x + w // 2, y + h // 2
            else:
                self.bbox = None
                self.position = (None, None)
        else:
            self.position = (None, None)
        self.image_detection_data['bbox'] = self.bbox
        self.image_detection_data['position'] = self.position
        return self.position

    def extract_yellow(self, frame):
        """
        Extracts yellow areas (like emergency vests) from the frame.
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # # Define range for yellow color (very tolerant)
        # lower_color = np.array([15, 80, 80])
        # upper_color = np.array([40, 255, 255])

        # # Pink (magenta) range — adjust if needed
        lower_color = np.array([140, 80, 80])
        upper_color = np.array([170, 255, 255])

        # Rust color range (reddish-brown/orange-brown)
        # lower_color = np.array([5, 100, 50])
        # upper_color = np.array([20, 255, 200])
        
        # Rust color range (Light Grayish-Blue)
        # lower_color = np.array([85, 10, 120])
        # upper_color = np.array([115, 50, 200])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Improve mask for blurred images
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)  # Smooth the mask

        return mask


def main():
    global CENTERED_X, CENTERED_Y
    model = ColorImageDetectionModel(reference_image_path="")

    cap = cv2.VideoCapture(0)  # Open default webcam
    # print dimensions of the camera
    print("Camera dimensions:")
    print("Width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    CENTERED_X = cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2
    CENTERED_Y = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // 2
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if model.detect_target(frame):
            pos = model.locate_target(frame)
            print(f"Target located at (relative): {pos}")
        else:
            pos = model.locate_target(frame)
            print("No color detected.")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# def main():
#     global CENTERED_X, CENTERED_Y
#     model = ColorImageDetectionModel(reference_image_path="", display=True)

#     # Replace this with the path to your video file
#     video_path = r"C:\Users\TLP-001\Desktop\TamirTalpi\שנה ב\מגדד\videos\vid1.mp4"
#     cap = cv2.VideoCapture(video_path)

#     if not cap.isOpened():
#         print(f"Error: Cannot open video file {video_path}")
#         return

#     #start the video from 10 seconds
#     cap.set(cv2.CAP_PROP_POS_MSEC, 12000)  # 10 seconds in milliseconds

#     # Print dimensions of the video
#     print("Video dimensions:")
#     print("Width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#     print("Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

#     CENTERED_X = cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2
#     CENTERED_Y = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // 2

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("End of video or cannot read the frame.")
#             break

#         if model.detect_target(frame):
#             pos = model.locate_target(frame)
#             print(f"Target located at (relative): {pos}")
#         else:
#             pos = model.locate_target(frame)
#             print("No color detected.")

#         if cv2.waitKey(30) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()


# if __name__ == "__main__":
#     main()
