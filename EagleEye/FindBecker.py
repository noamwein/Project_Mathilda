import cv2
import numpy as np
import face_recognition

def fast_face_check(roi_gray):
    # Load a pre-trained Haar cascade or DNN face detector
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    
    # Perform face detection in the region of interest
    faces = face_cascade.detectMultiScale(roi_gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    return faces

def find_face_in_frame(frame, reference_encoding, frame_skip=3, scale=0.5, tolerance=0.9):
    # Resize frame using NumPy for faster processing
    small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Detect all faces in the resized frame and their encodings
    face_locations = face_recognition.face_locations(rgb_small_frame, model="hog")  # Use 'hog' for speed
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    if not face_encodings:
        return False

    # Convert to NumPy arrays for efficient matching
    reference_encoding_np = np.array(reference_encoding)

    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Compute the distance directly instead of compare_faces
        distances = np.linalg.norm(reference_encoding_np - face_encoding, axis=0)

        # Check if the match is within the tolerance
        if distances <= tolerance:
            # Scale up face location to match the original frame size
            top, right, bottom, left = face_location
            top, right, bottom, left = (
                int(top / scale),
                int(right / scale),
                int(bottom / scale),
                int(left / scale),
            )
            return left, right, top, bottom

    return False

