server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print(f"Server listening on {HOST}:{PORT}")

conn, addr = server_socket.accept()
print(f"Connection from {addr}")

data = b''
payload_size = struct.calcsize("Q")

try:
    while True:
        # Receive message size first
        while len(data) < payload_size:
            packet = conn.recv(4096)
            if not packet:
                break
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        # Receive the actual frame data
        while len(data) < msg_size:
            data += conn.recv(4096)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        # Deserialize the frame
        frame = pickle.loads(frame_data)

        # Display the frame
        cv2.imshow('Server Webcam', frame)
        if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()
