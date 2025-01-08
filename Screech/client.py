import socket
import typing

from Screech.request_handler import RequestHandler


class Client:
    def __init__(self, host: str, port: int, request_handler: RequestHandler = None):
        self.host = host
        self.port = port
        self.request_handler = request_handler or RequestHandler()

    def send_request(self, request: typing.Any) -> typing.Any:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((self.host, self.port))

            encoded_request = self.request_handler.encode_input(request)
            client_socket.sendall(encoded_request)
            print(f"Sent request: {request}")

            response_data = client_socket.recv(1024)
            decoded_response = self.request_handler.decode_output(response_data)
            print(f"Received response: {decoded_response}")
            return decoded_response
