import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

import cv2

from EagleEye.sources.picamera_source import PiCameraSource


def main():
    camera_source = PiCameraSource()

    try:
        while True:
            frame = camera_source.get_current_frame()
            if frame is not None:
                cv2.imshow("Live Camera Feed", frame)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
