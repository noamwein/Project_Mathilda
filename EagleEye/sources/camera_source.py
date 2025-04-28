import cv2

from BirdBrain.interfaces import Source


class CameraSource(Source):
    def __init__(self, camera_index=0, retries=3, *args, **kwargs):
        """
        Initialize the camera source.

        :param camera_index: Index of the camera to use (default: 0 for the first camera).
        """
        super().__init__(*args, **kwargs)

        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            print('Failed to open camera. Retrying...')
            for attempt in range(retries):
                print(f"Retrying to open the camera (Attempt {attempt + 1}/{retries})...")
                self.camera = cv2.VideoCapture(camera_index)
                if self.camera.isOpened():
                    print("Camera successfully opened.")
                    return
            raise Exception("Failed to open the camera after multiple attempts.")

    def _get_current_frame(self):
        """
        Capture the current frame from the camera.

        :return: The captured frame as a numpy array, or None if unsuccessful.
        """
        ret, frame = self.camera.read()
        if not ret:
            return None
        return frame

    def release(self):
        """
        Release the camera resource.
        """
        self.camera.release()
