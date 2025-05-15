import cv2
import numpy as np


def _resize_and_pad(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    """
    Resize an image to fit into a target frame while maintaining aspect ratio,
    and pad with black pixels to match the exact target size.

    Parameters:
        frame (np.ndarray): The input image.
        target_width (int): Target frame width.
        target_height (int): Target frame height.

    Returns:
        np.ndarray: The resized and padded image.
    """
    original_height, original_width = frame.shape[:2]

    # Compute the scaling factor to fit the image into the target frame
    scale = min(target_width / original_width, target_height / original_height)

    # Resize image while maintaining aspect ratio
    new_width = int(original_width * scale)
    new_height = int(original_height * scale)
    resized_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)

    # Create a black canvas of target size
    padded_frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)

    # Compute top-left corner to place the resized image
    x_offset = (target_width - new_width) // 2
    y_offset = (target_height - new_height) // 2

    # Place the resized image onto the canvas
    padded_frame[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized_frame

    return padded_frame
