import cv2
import numpy as np
import face_recognition
import FindBecker

# CONSTANTS
START_FROM = 60
PROCESS_EVERY = 5  # Process face recognition every X frames
# tracking object algorithm:
MIN_TRACKING_POINTS = 4
DISTANCE_THRESHOLD = 1000  # ignore points that are too far from the average
PADDING = 150  # increase the interesting rectangle by this padding

# Global variables
clicked_point = False  # To detect clicks
face_found = False
face_center = None
bbox_size = 50
p0 = None
old_gray = None
mask_lines = None

# Paths
PATH = r"G:\My Drive\magdad_matilda\image_processing_data\beker"
video_path = PATH + r"\beker_from_drone.mp4"
reference_image_path = PATH + r"\b6.png"

# Load the reference image and compute its face encoding
reference_image = face_recognition.load_image_file(reference_image_path)
reference_encodings = face_recognition.face_encodings(reference_image)
if len(reference_encodings) == 0:
    raise ValueError("No face found in the reference image.")
reference_encoding = reference_encodings[0]


def mouse_callback(event, x, y, flags, param):
    """Mouse callback to detect clicks."""
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = True  # Set flag to trigger re-detection


def follow_face(video_source=0):
    global clicked_point, face_found, face_center, bbox_size, p0, old_gray, mask_lines

    # Initialize the video capture object
    video_capture = cv2.VideoCapture(video_source)
    video_capture.set(cv2.CAP_PROP_POS_MSEC, START_FROM * 1000)

    face_found = False
    face_center = None
    old_gray = None
    p0 = None
    frame_counter = 0  # Initialize the frame counter

    # Parameters for Shi-Tomasi corner detection
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    while True:
        # Capture a single frame of video
        ret, frame = video_capture.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Convert the frame to grayscale for optical flow calculations
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_counter += 1  # Increment the frame counter

        if not face_found:
            # Only process face recognition every X frames
            if frame_counter % PROCESS_EVERY == 0:
                face_center = FindBecker.find_face_in_frame(frame, reference_encoding, frame_skip=3, scale=0.5,
                                                            tolerance=0.6)
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

                if len(good_new) > 0:
                    # Calculate the average position of the points
                    avg_x = np.mean(good_new[:, 0])
                    avg_y = np.mean(good_new[:, 1])

                    cv2.circle(frame, (int(avg_x), int(avg_y)), 25, (255, 0, 0), -1)

                    # Define a distance threshold (e.g., max 50 pixels from the centroid)
                    distances = np.sqrt((good_new[:, 0] - avg_x) ** 2 + (good_new[:, 1] - avg_y) ** 2)

                    # Filter points based on the threshold
                    filtered_points = good_new[distances <= DISTANCE_THRESHOLD]

                    if len(filtered_points) > MIN_TRACKING_POINTS:
                        # Update the bounding box using filtered points
                        x_min = int(np.min(filtered_points[:, 0]))
                        y_min = int(np.min(filtered_points[:, 1]))
                        x_max = int(np.max(filtered_points[:, 0]))
                        y_max = int(np.max(filtered_points[:, 1]))
                        # increasing the rect a little bit
                        x_min -= PADDING
                        y_min -= PADDING
                        x_max += PADDING
                        y_max += PADDING
                        # Draw the bounding box
                        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 10)

                        # Draw the tracking points
                        for point in filtered_points:
                            cv2.circle(frame, (int(point[0]), int(point[1])), 8, (0, 0, 255), -1)
                        ##############
                        # Detect new interesting points within the updated bounding box
                        region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
                        new_points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **feature_params)

                        if new_points is not None:
                            # Adjust new points to the original frame's coordinate system
                            new_points[:, 0, 0] += x_min
                            new_points[:, 0, 1] += y_min

                            # Merge new points with existing ones, avoiding duplicates
                            if p0 is not None:
                                all_points = np.vstack((p0, new_points))
                            else:
                                all_points = new_points

                            # Filter duplicate points (optional, based on proximity)
                            unique_points = []
                            for point in all_points[:, 0, :]:
                                if len(unique_points) == 0 or not any(
                                        np.linalg.norm(point - np.array(pt)) < 5 for pt in unique_points):
                                    unique_points.append(point)
                            p0 = np.array(unique_points, dtype=np.float32).reshape(-1, 1, 2)
                        else:
                            p0 = filtered_points.reshape(-1, 1, 2)

                        # Update the previous gray frame
                        old_gray = gray_frame
                        ##############
                        p0 = filtered_points.reshape(-1, 1, 2)
                    else:
                        face_found = False  # Re-detect if no points are left
                else:
                    face_found = False  # Re-detect if tracking fails

        # Display the resulting frame
        frame = cv2.resize(frame, (800, 450))
        cv2.imshow('Video', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all windows
    video_capture.release()
    cv2.destroyAllWindows()


# Run the function with the video path
follow_face(video_path)
