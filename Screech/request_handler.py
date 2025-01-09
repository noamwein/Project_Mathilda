import abc
import pickle
import sys
import typing
import socket
import struct
import numpy as np


def send_data(socket_conn: socket.socket, data: bytes):
    """Send data by first sending the size and then the serialized data."""
    data_size = len(data)

    # Send the size of the data (using struct to handle fixed size for the length)
    socket_conn.sendall(struct.pack("!I", data_size))  # !I is for an unsigned int in network byte order
    socket_conn.sendall(data)


def receive_data(socket_conn: socket.socket) -> bytes:
    """Receive data, first receiving the size and then the actual data."""
    # Receive the size of the incoming message
    size_data = socket_conn.recv(4)  # 4 bytes for the size of the message
    if len(size_data) < 4:
        raise ValueError("Error: size data truncated.")

    data_size = struct.unpack("!I", size_data)[0]  # Unpack the size (network byte order to native)

    # Receive the actual data
    remaining_data = b''
    while len(remaining_data) < data_size:
        chunk = socket_conn.recv(min(4096, data_size - len(remaining_data)))  # Read the remaining bytes
        if not chunk:
            raise ValueError("Error: Received unexpected end of data.")
        remaining_data += chunk

    return remaining_data


def receive_with_timeout(sock, timeout):
    """
    Receives data from a socket with a specified timeout.

    Args:
        sock (socket.socket): The socket object to receive data from.
        timeout (float): Timeout duration in seconds. Set to None for no timeout.

    Returns:
        bytes: The received data if successful.
        None: If no data is received before the timeout.

    Raises:
        ValueError: If packet_size is not a positive integer.
    """
    # Set the timeout for the socket
    sock.settimeout(timeout)

    try:
        # Attempt to receive data from the socket
        data = receive_data(sock)
        return data
    except socket.timeout:
        print("Socket timed out after waiting for", timeout, "seconds")
        return None
    except socket.error as e:
        print("Socket error occurred:", e)
        return None
    finally:
        # Reset the timeout to None (blocking mode) after the operation
        sock.settimeout(None)


def array_to_bytes(array: np.ndarray) -> bytes:
    """Convert a NumPy array to bytes, including dtype and shape."""
    # Convert the dtype and shape into bytes
    dtype_str = array.dtype.str  # dtype as a string
    shape = array.shape  # tuple of dimensions
    shape_bytes = struct.pack('I' * len(shape), *shape)  # Convert shape into binary format

    # Convert the array data into bytes
    array_bytes = array.tobytes()

    # Concatenate the dtype, shape, and array data
    return dtype_str.encode() + struct.pack('I', len(dtype_str)) + shape_bytes + array_bytes


def bytes_to_array(data: bytes) -> np.ndarray:
    """Convert bytes back into a NumPy array, extracting dtype and shape."""
    # Extract the dtype string length and dtype string
    dtype_len = struct.unpack('I', data[4:8])[0]  # dtype length is packed after the first 4 bytes
    dtype_str = data[8:8 + dtype_len].decode()

    # Extract the shape of the array
    shape_start = 8 + dtype_len
    shape_size = (len(data) - shape_start - dtype_len) // 4  # Number of elements in shape
    shape = struct.unpack('I' * shape_size, data[shape_start:shape_start + 4 * shape_size])

    # Extract the actual array bytes
    array_bytes = data[shape_start + 4 * shape_size:]

    # Convert the byte data back into the numpy array with the correct dtype and shape
    return np.frombuffer(array_bytes, dtype=dtype_str).reshape(shape)


class RequestHandler(abc.ABC):
    def handle_request(self, request):
        pass

    def encode_input(self, msg: typing.Any) -> bytes:
        if isinstance(msg, np.ndarray):
            return array_to_bytes(msg)
        return pickle.dumps(msg)

    def decode_input(self, encoded_msg: bytes) -> typing.Any:
        try:
            return bytes_to_array(encoded_msg)
        except Exception:
            print('exception: ', sys.exc_info()[0])
            return pickle.loads(encoded_msg)

    def encode_output(self, msg: typing.Any) -> bytes:
        return pickle.dumps(msg)

    def decode_output(self, encoded_msg: bytes) -> typing.Any:
        return pickle.loads(encoded_msg)
