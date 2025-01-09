import socket
import typing

from Screech.request_handler import RequestHandler, send_data, receive_with_timeout


class Client:
    def __init__(self, host: str, port: int, request_handler: RequestHandler = None, timeout: int = 10):
        self.host = host
        self.port = port
        self.request_handler = request_handler or RequestHandler()
        self.timeout = timeout

    def send_request(self, request: typing.Any) -> typing.Any:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((self.host, self.port))

            encoded_request = self.request_handler.encode_input(request)
            send_data(client_socket, encoded_request)
            print(f"Sent request: {request}")

            response_data = receive_with_timeout(client_socket, timeout=self.timeout)
            if response_data is None:
                return None

            decoded_response = self.request_handler.decode_output(response_data)
            print(f"Received response: {decoded_response}")
            return decoded_response
