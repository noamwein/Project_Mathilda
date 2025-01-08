import socket
import cv2
import numpy as np
import argparse
from config import SERVER_PORT, MAX_PACKET_SIZE

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Server for receiving webcam video over WiFi.")
parser.add_argument('--port', type=int, default=SERVER_PORT, help="Port number to listen on. Default is defined in config.py.")
args = parser.parse_args()

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = args.port

# Create a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

print(f"Server listening on {HOST}:{PORT}")

try:
    while True:
        # Receive the data from the client
        packet, addr = server_socket.recvfrom(MAX_PACKET_SIZE)

        # Decode the JPEG frame
        np_data = np.frombuffer(packet, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        # Display the frame
        cv2.imshow('Server Webcam', frame)
        if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    server_socket.close()
    cv2.destroyAllWindows()