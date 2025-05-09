import os
import sys
import cv2
import time
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from EagleEye.sources.picamera_source import PiCameraSource


def main():
    # Create the video output directory if it doesn't exist
    video_dir = os.path.join(os.path.dirname(__file__), '../../videos')
    os.makedirs(video_dir, exist_ok=True)

    # Create an instance of the camera source
    camera_source = PiCameraSource(logging=True)

    # Get the current timestamp for video filename
    timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    video_file = os.path.join(video_dir, f"{timestamp}.mp4")

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 file format
    frame = camera_source.get_current_frame()
    if frame is not None:
        # Initialize the VideoWriter with the frame's size
        video_writer = cv2.VideoWriter(video_file, fourcc, 30.0, (frame.shape[1], frame.shape[0]))

    try:
        while True:
            frame = camera_source.get_current_frame()

            if frame is not None:
                # Show the live feed in a window
                cv2.imshow("Live Camera Feed", frame)

                # Write the frame to the video file
                video_writer.write(frame)

            # Press 'q' to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Release the VideoWriter and destroy all OpenCV windows
        video_writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
