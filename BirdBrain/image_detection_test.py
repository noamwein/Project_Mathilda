import argparse
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from EagleEye.image_detection_models.ImageDetectionModel import ImageDetectionModel
from EagleEye.ImageProcessingConstants import *
from EagleEye.sources.video_source import VideoSource
from EagleEye.sources.picamera_source import PiCameraSource
from Screech.config import SERVER_IP, SERVER_PORT
from Screech.image_detection_client import RemoteImageDetection


def main():
    parser = argparse.ArgumentParser(description="Image Detection Script")
    parser.add_argument(
        '--video-path',
        type=str,
        default=VIDEO_PATH,
        help="Path to the video file."
    )
    parser.add_argument(
        '--image-path',
        type=str,
        default=REFERENCE_IMAGE_PATH,
        help="Path to the reference image."
    )
    parser.add_argument(
        '--disable-display',
        action='store_true',
        help="Disable to display the video feed."
    )
    parser.add_argument(
        '--recognize',
        action='store_true',
        help="Always try to recognize the person in the frame (for testing face recognition)."
    )
    parser.add_argument(
        '--remote',
        action='store_true',
        help="Run the computation on a remote server."
    )
    parser.add_argument(
        '--camera-source',
        action='store_true',
        help="Use the camera as the source."
    )
    parser.add_argument('--server-ip', type=str, default=SERVER_IP,
                        help="IP address of the server. Default is defined in config.py.")
    parser.add_argument('--server-port', type=int, default=SERVER_PORT,
                        help="Port number of the server. Default is defined in config.py.")

    args = parser.parse_args()

    model = ImageDetectionModel(reference_image_path=args.image_path, display=not args.disable_display,
                                always_recognize_person=args.recognize)
    if args.remote:
        model = RemoteImageDetection(image_detection_model=model, server_host=args.server_ip,
                                     server_port=args.server_port)
    if args.camera_source:
        video_source = PiCameraSource()
    else:
        video_source = VideoSource(args.video_path, START_FROM_SECONDS)

    done = False
    while not done:
        target_position = model.locate_target(video_source.get_current_frame())
        #print(target_position)
        # Uncomment and implement drone client logic if needed
        # if self.drone_client.is_on_target(target_position):
        #     if self.drone_client.has_stopped():
        #         self.assassinate()
        #     else:
        #         self.drone_client.stop_movement()
        # else:
        #     self.drone_client.goto_target(target_position)


if __name__ == '__main__':
    main()
