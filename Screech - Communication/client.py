import cv2
import socket
import pickle
import argparse
from config import SERVER_IP, SERVER_PORT, MAX_PACKET_SIZE

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Client for streaming webcam video over WiFi.")
parser.add_argument('--server_ip', type=str, default=SERVER_IP, help="IP address of the server. Default is defined in config.py.")
parser.add_argument('--server_port', type=int, default=SERVER_PORT, help="Port number of the server. Default is defined in config.py.")
parser.add_argument('--display', action='store_true', help="Run in display mode to show the webcam feed locally. Defaults to headless mode.")
args = parser.parse_args()

SERVER_IP = args.server_ip
SERVER_PORT = args.server_port
DISPLAY_MODE = args.display

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initialize webcam capture
camera = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            break

        # Compress the frame using JPEG to fit within the max packet size
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # Adjust quality as needed
        _, compressed_frame = cv2.imencode('.jpg', frame, encode_param)
        data = compressed_frame.tobytes()

        # Ensure the data fits within the MAX_PACKET_SIZE
        if len(data) > MAX_PACKET_SIZE:
            print("Warning: Frame size exceeds MAX_PACKET_SIZE. Dropping frame.")
            continue

        # Send the compressed frame to the server
        client_socket.sendto(data, (SERVER_IP, SERVER_PORT))

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