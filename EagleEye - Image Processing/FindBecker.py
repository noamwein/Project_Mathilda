import cv2
import face_recognition

def find_face_in_frame(frame, reference_encoding, frame_skip=3, scale=0.5, tolerance=0.9):
    # Resize frame to speed up processing (adjust scale for balance)
    small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Detect all faces in the resized frame and their encodings
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Compare this face to the reference encoding
        match = face_recognition.compare_faces([reference_encoding], face_encoding, tolerance=tolerance)

        # If the reference image matches
        if match[0]:
            # Scale up face location to match the original frame size
            top, right, bottom, left = face_location
            top, right, bottom, left = top // scale, right // scale, bottom // scale, left // scale

            # Calculate the middle of the face
            middle_x = (left + right) // 2
            middle_y = (top + bottom) // 2

            return (middle_x, middle_y)

    return False