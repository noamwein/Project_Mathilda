import cv2

from EagleEye.image_detection_models.color_detection_model import CENTERED_X, CENTERED_Y, ImageDetection
from EagleEye.image_detection_models.color_detection_model import ColorImageDetectionModel
from EagleEye.sources.camera_source import CameraSource
from Monitor.video_saver import VideoSaver
from Rogatka.drone_client import DroneClient
from Rogatka.dummy_client import DummyClient


class GUI:
    def __init__(self, drone_client: DroneClient, video_saver: VideoSaver, image_detection: ImageDetection,
                 enable_display=True):
        self.drone_client = drone_client
        self.video_saver = video_saver
        self.enable_display = enable_display
        self.image_detection = image_detection

        # Create a named window
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

        # Set the window to fullscreen
        cv2.setWindowProperty("Video", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def draw_gui(self, frame):
        if frame is None:
            return
        processed_frame = self._draw_gui(frame)
        if self.video_saver is not None:
            self.video_saver.write_frame(processed_frame)
        if self.enable_display:
            cv2.imshow('Video', processed_frame)

    def _draw_gui(self, frame):
        bbox = self.image_detection.image_detection_data.get('bbox')
        processed_frame = frame.copy()
        self.draw_cross(processed_frame)
        if bbox:
            self.draw_bounding_box(frame, bbox)
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


def main():
    source = CameraSource()
    gui = GUI(drone_client=DummyClient(), video_saver=VideoSaver(), image_detection=ColorImageDetectionModel(None))
    while True:
        frame = source.get_current_frame()
        gui.draw_gui(frame)


if __name__ == '__main__':
    main()
