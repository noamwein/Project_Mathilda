import cv2
import socket
import struct
import pickle

# Define the server address and port
SERVER_IP = '192.168.1.100'  # Replace with your server's IP
SERVER_PORT = 9999

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

        # Display the frame locally (optional)
        cv2.imshow('Client Webcam', frame)
        if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    camera.release()
    client_socket.close()
    cv2.destroyAllWindows()
