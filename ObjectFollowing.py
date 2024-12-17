import FindBecker
import cv2
import numpy as np
import face_recognition

# Path to the reference image
reference_image_path = "images/Noam_Weingarten.jpg"

# Load the reference image and compute its face encoding
reference_image = face_recognition.load_image_file(reference_image_path)
reference_encodings = face_recognition.face_encodings(reference_image)
if len(reference_encodings) == 0:
    raise ValueError("No face found in the reference image.")
reference_encoding = reference_encodings[0]

def follow_face(video_source=0):
    # Initialize the video capture object
    video_capture = cv2.VideoCapture(video_source)

    face_found = False
    face_center = None
    old_gray = None
    p0 = None

    # Parameters for ShiTomasi corner detection
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    while True:
        # Capture a single frame of video
        ret, frame = video_capture.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Convert the frame to grayscale for optical flow calculations
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if not face_found:
            # Use FindBecker to locate the face in the frame
            face_center = FindBecker.find_face_in_frame(frame, reference_encoding, frame_skip=3, scale=0.5, tolerance=0.9)
            if face_center:
                face_found = True
                middle_x, middle_y = face_center

                # Define an initial bounding box around the face
                bbox_size = 50  # Approximate size of the bounding box (can be adjusted)
                left = max(0, int(middle_x - bbox_size))
                right = min(frame.shape[1], int(middle_x + bbox_size))
                top = max(0, int(middle_y - bbox_size))
                bottom = min(frame.shape[0], int(middle_y + bbox_size))

                # Initialize tracking points within the bounding box
                face_region = gray_frame[top:bottom, left:right]
                p0 = cv2.goodFeaturesToTrack(face_region, mask=None, **feature_params)

                if p0 is not None:
                    # Adjust points to the original frame's coordinate system
                    p0[:, 0, 0] += left
                    p0[:, 0, 1] += top
                old_gray = gray_frame
        else:
            # Track the face using optical flow
            if p0 is not None and old_gray is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, p0, None, **lk_params)

                # Select good points
                good_new = p1[st == 1]
                good_old = p0[st == 1]

                # If there are enough points, calculate a new bounding box
                if len(good_new) > 0:
                    x_min = int(np.min(good_new[:, 0]))
                    y_min = int(np.min(good_new[:, 1]))
                    x_max = int(np.max(good_new[:, 0]))
                    y_max = int(np.max(good_new[:, 1]))
                    face_center = ((x_min + x_max) // 2, (y_min + y_max) // 2)
                else:
                    face_found = False  # Revert to detection if tracking fails

                # Update points for the next iteration
                old_gray = gray_frame
                p0 = good_new.reshape(-1, 1, 2)

        if face_center:
            # Draw a rectangle around the face
            middle_x, middle_y = face_center
            left = max(0, int(middle_x - bbox_size))  # Cast to int
            right = min(frame.shape[1], int(middle_x + bbox_size))  # Cast to int
            top = max(0, int(middle_y - bbox_size))  # Cast to int
            bottom = min(frame.shape[0], int(middle_y + bbox_size))  # Cast to int
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Video', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all windows
    video_capture.release()
    cv2.destroyAllWindows()

# Run the function
follow_face()
