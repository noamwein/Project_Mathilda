import os.path

import cv2

from BirdBrain.interfaces import VideoSaver
from EagleEye.sources.camera_source import CameraSource
import datetime

VIDEO_DIR_PATH = os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, 'videos')


class MP4VideoSaver(VideoSaver):
    def __init__(self, fps=30):
        self.fps = fps
        timestamp = datetime.datetime.now().strftime('%d-%m-%Y %H%M%S')
        self.video_path = os.path.join(VIDEO_DIR_PATH, f'{timestamp}.mp4')
        self.out = None

    def write_frame(self, frame):
        if self.out is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter(self.video_path, fourcc, self.fps, (width, height))
            if not self.out.isOpened():
                raise IOError(f"Cannot open video file for writing: {self.video_path}")
        self.out.write(frame)

    def save_and_close(self):
        if self.out:
            self.out.release()
            print(f"Video saved to {self.video_path}")
        else:
            print("No frames were written. Video not saved.")


def main():
    saver = MP4VideoSaver(fps=30)
    source = CameraSource()
    for _ in range(2 * 30):  # 2 seconds at 30 FPS
        frame = source.get_current_frame()
        saver.write_frame(frame)

    saver.save_and_close()


if __name__ == "__main__":
    main()
