import abc
import pickle
import typing


class RequestHandler(abc.ABC):
    def handle_request(self, request):
        pass

    def encode_input(self, msg: typing.Any) -> bytes:
        return pickle.dumps(msg)

    def decode_input(self, encoded_msg: bytes) -> typing.Any:
        return pickle.loads(encoded_msg)

    def encode_output(self, msg: typing.Any) -> bytes:
        return pickle.dumps(msg)

    def decode_output(self, encoded_msg: bytes) -> typing.Any:
        return pickle.loads(encoded_msg)
