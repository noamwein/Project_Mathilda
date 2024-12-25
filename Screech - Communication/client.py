import cv2
import socket
import struct
import pickle
import argparse
from config import SERVER_IP, SERVER_PORT

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Client for streaming webcam video over WiFi.")
parser.add_argument('--server_ip', type=str, default=SERVER_IP, help="IP address of the server. Default is defined in config.py.")
parser.add_argument('--server_port', type=int, default=SERVER_PORT, help="Port number of the server. Default is defined in config.py.")
parser.add_argument('--display', action='store_true', help="Run in display mode to show the webcam feed locally. Defaults to headless mode.")
args = parser.parse_args()

SERVER_IP = args.server_ip
SERVER_PORT = args.server_port
DISPLAY_MODE = args.display

# Create a socket connection
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((SERVER_IP, SERVER_PORT))

# Initialize webcam capture
camera = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            break

        # Serialize the frame
        data = pickle.dumps(frame)
        # Send frame size first, followed by the actual frame data
        client_socket.sendall(struct.pack("Q", len(data)) + data)

        # Display the frame locally if in display mode
        if DISPLAY_MODE:
            cv2.imshow('Client Webcam', frame)
            if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
                break

except Exception as e:
    print(f"Error: {e}")

finally:
    camera.release()
    client_socket.close()
    if DISPLAY_MODE:
        cv2.destroyAllWindows()