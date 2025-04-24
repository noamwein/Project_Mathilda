import cv2

from BirdBrain.interfaces import Source


class VideoSource(Source):
    def __init__(self, video_path: str, start_time: int = 0):
        self.capture = cv2.VideoCapture(video_path)
        self.capture.set(cv2.CAP_PROP_POS_MSEC, start_time * 1000)
        # print("width:", self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        # print("height:", self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # print("fps:", self.capture.get(cv2.CAP_PROP_FPS))

    def get_current_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            raise ValueError("Failed to capture frame from video source.")
        return frame

    def release(self):
        self.capture.release()
