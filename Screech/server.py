import argparse
import socket
import threading
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from Screech.request_handler import RequestHandler, send_data, receive_data
from Screech.config import SERVER_PORT
from Screech.image_detection_client import ImageDetectionRequestHandler
from EagleEye.ImageDetectionModel import ImageDetectionModel
from EagleEye.ImageProcessingConstants import REFERENCE_IMAGE_PATH


class Server:
    def __init__(self, port, request_handler: RequestHandler):
        self.request_handler = request_handler
        self.port = port
        self.running = False

    def run_server(self):
        self.running = True
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', self.port))
        server_socket.listen(5)
        print(f"Server running on port {self.port}")

        try:
            while self.running:
                client_socket, client_address = server_socket.accept()
                print(f"Connection accepted from {client_address}")
                threading.Thread(target=self._handle_client, args=(client_socket,)).start()
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.running = False
            server_socket.close()

    def _handle_client(self, client_socket):
        with client_socket:
            try:
                data = receive_data(client_socket)

                decoded_request = self.request_handler.decode_input(data)
                print(f"Received request: {decoded_request}")

                response = self.request_handler.handle_request(decoded_request)
                encoded_response = self.request_handler.encode_output(response)
                send_data(client_socket, encoded_response)
                print(f"Sent response: {response}")

            except Exception as e:
                print(f"Error handling client: {e}")


def main():
    parser = argparse.ArgumentParser(description="Server for handling requests")
    parser.add_argument('--port', type=int, default=SERVER_PORT,
                        help="Port number to listen on. Default is defined in config.py.")
    parser.add_argument(
        '--image-path',
        type=str,
        default=REFERENCE_IMAGE_PATH,
        help="Path to the reference image."
    )
    args = parser.parse_args()

    model = ImageDetectionModel(reference_image_path=args.image_path, display=False)
    server = Server(port=args.port, request_handler=ImageDetectionRequestHandler(model))
    server.run_server()


if __name__ == '__main__':
    main()
