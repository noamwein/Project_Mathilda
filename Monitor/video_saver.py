import os
from datetime import datetime
import cv2


class VideoSaver:
    def __init__(self):
        self.video_writer = None

    def init_recording(self, frame):
        # Create the video output directory if it doesn't exist
        video_dir = os.path.join(os.path.dirname(__file__), '../../videos')
        os.makedirs(video_dir, exist_ok=True)

        # Get the current timestamp for video filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        video_file = os.path.join(video_dir, f"{timestamp}.mp4")

        # Define the codec and create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 file format
        # Initialize the VideoWriter with the frame's size
        self.video_writer = cv2.VideoWriter(video_file, fourcc, 30.0, (frame.shape[1], frame.shape[0]))

    def write_frame(self, frame):
        if self.video_writer is None:
            self.init_recording(frame)
        self.video_writer.write(frame)

    def save_and_close(self):
        if self.video_writer:
            print("Closing video writer...")
            self.video_writer.release()
