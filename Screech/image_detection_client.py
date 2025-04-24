import enum
import traceback
from typing import Tuple

from BirdBrain.interfaces import ImageDetection
from EagleEye.image_detection_models.ImageDetectionModel import ImageDetectionModel
from Screech.client import Client
from Screech.request_handler import RequestHandler


class Actions(enum.Enum):
    DETECT = 0
    LOCATE = 1


class ImageDetectionRequestHandler(RequestHandler):
    def __init__(self, image_detection_model):
        self.image_detection_model = image_detection_model

    def handle_request(self, request):
        action, frame = request
        if action == Actions.DETECT:
            # detect person
            return self.image_detection_model.detect_target(frame)
        elif action == Actions.LOCATE:
            # locate person
            return self.image_detection_model.locate_target(frame)


class RemoteImageDetection(ImageDetection):
    def __init__(self, image_detection_model: ImageDetectionModel, server_host, server_port):
        self.image_detection_model = image_detection_model
        self.client = Client(host=server_host, port=server_port)

    def detect_target(self, frame) -> bool:
        try:
            print('waiting for server to detect target')
            request = (Actions.DETECT, frame)
            response = self.client.send_request(request)
            if response is not None:
                print('received response: {}'.format(response))
                return response
            return self.image_detection_model.detect_target(frame)
        except Exception:
            print('exception: {}'.format(traceback.format_exc()))
            print('running locally')
            return self.image_detection_model.detect_target(frame)

    def locate_target(self, frame) -> Tuple[int, int]:
        try:
            print('waiting for server to locate target')
            request = (Actions.LOCATE, frame)
            response = self.client.send_request(request)
            if response is not None:
                print('received response: {}'.format(response))
                return response
            return self.image_detection_model.locate_target(frame)
        except Exception:
            print('exception: {}'.format(traceback.format_exc()))
            print('running locally')
            return self.image_detection_model.locate_target(frame)
