import cv2
import numpy as np
import face_recognition
import FindBecker

# CONSTANTS
START_FROM_SECONDS = 60  # Video start time in seconds
PROCESS_EVERY_FRAMES = 5  # Process face recognition every X frames
MIN_TRACKING_POINTS = 4
DISTANCE_THRESHOLD = 150
PADDING_PIXELS = 0
INITIAL_BBOX_SIZE = 150  # Initial bounding box size around detected face
FRAME_WIDTH = 800  # Display frame width
FRAME_HEIGHT = 450  # Display frame height

# Paths
PATH = r"G:\My Drive\magdad_matilda\image_processing_data\beker"
VIDEO_PATH = PATH + r"\beker_from_drone.mp4"
VIDEO_PATH = PATH + r"\beker_from_drone.mp4"
REFERENCE_IMAGE_PATH = PATH + r"\b6.png"

# Parameters for Shi-Tomasi corner detection
FEATURE_PARAMS = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

# Parameters for Lucas-Kanade optical flow
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


def load_reference_encoding(reference_image_path):
    reference_image = face_recognition.load_image_file(reference_image_path)
    reference_encodings = face_recognition.face_encodings(reference_image)
    if not reference_encodings:
        raise ValueError("No face found in the reference image.")
    return reference_encodings[0]


def initialize_video_capture(video_path, start_time):
    capture = cv2.VideoCapture(video_path)
    capture.set(cv2.CAP_PROP_POS_MSEC, start_time * 1000)
    return capture


def find_face(frame, reference_encoding):
    return FindBecker.find_face_in_frame(frame, reference_encoding, frame_skip=3, scale=0.5, tolerance=0.6)


def initialize_tracking_points(gray_frame, bbox, feature_params):
    x_min, y_min, x_max, y_max = bbox
    region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
    points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **feature_params)
    if points is not None:
        points[:, 0, 0] += x_min
        points[:, 0, 1] += y_min
    return points


def update_tracking_points(old_gray, gray_frame, points, lk_params):
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, points, None, **lk_params)
    good_new = p1[st == 1] if p1 is not None else None
    return good_new


def filter_points(points):
    if len(points) == 0:
        return points
    points = points.reshape(-1, 2)  # Reshape to (N, 2) for easier calculations
    avg_x = np.mean(points[:, 0])
    avg_y = np.mean(points[:, 1])
    # avg_x = points[0, 0]
    # avg_y = points[0, 1]
    distances = np.sqrt((points[:, 0] - avg_x) ** 2 + (points[:, 1] - avg_y) ** 2)
    filtered_points = points[distances <= DISTANCE_THRESHOLD]
    return filtered_points.reshape(-1, 1, 2), avg_x, avg_y  # Reshape back to (N, 1, 2)


def draw_tracking(frame, points, avg_x, avg_y):
    for point in points:
        cv2.circle(frame, (int(point[0][0]), int(point[0][1])), 8, (0, 0, 255), -1)
    cv2.circle(frame, (int(avg_x), int(avg_y)), 25, (255, 0, 0), -1)


def draw_bounding_box(frame, bbox):
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 10)


def add_more_interesting_points(gray_frame, bbox, points, feature_params):
    x_min, y_min, x_max, y_max = bbox
    region_of_interest = gray_frame[y_min:y_max, x_min:x_max]
    new_points = cv2.goodFeaturesToTrack(region_of_interest, mask=None, **feature_params)
    if new_points is not None:
        new_points[:, 0, 0] += x_min
        new_points[:, 0, 1] += y_min
        # max_new_points = len(points) // 4 if points is not None else new_points.shape[0]
        # max_new_points = 1
        # new_points = new_points[:max_new_points]
        points = np.vstack((points, new_points)) if points is not None else new_points
    return points


def follow_face(video_path, reference_encoding):
    video_capture = initialize_video_capture(video_path, START_FROM_SECONDS)
    old_gray, points = None, None
    face_found, frame_counter = False, 0

    while True:
        ret, frame = video_capture.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_counter += 1

        if not face_found and frame_counter % PROCESS_EVERY_FRAMES == 0:
            face_center = find_face(frame, reference_encoding)
            if face_center:
                face_found = True
                x, y = face_center
                bbox = (max(0, int(x - INITIAL_BBOX_SIZE)), max(0, int(y - INITIAL_BBOX_SIZE)),
                        min(frame.shape[1], int(x + INITIAL_BBOX_SIZE)),
                        min(frame.shape[0], int(y + INITIAL_BBOX_SIZE)))
                points = initialize_tracking_points(gray_frame, bbox, FEATURE_PARAMS)
                old_gray = gray_frame

        elif face_found and points is not None:
            points = update_tracking_points(old_gray, gray_frame, points, LK_PARAMS)
            filtered_points, avg_x, avg_y = filter_points(points)
            if filtered_points is not None and len(filtered_points) > MIN_TRACKING_POINTS:
                bbox = (int(np.min(filtered_points[:, 0, 0])) - PADDING_PIXELS,
                        int(np.min(filtered_points[:, 0, 1])) - PADDING_PIXELS,
                        int(np.max(filtered_points[:, 0, 0])) + PADDING_PIXELS,
                        int(np.max(filtered_points[:, 0, 1])) + PADDING_PIXELS)
                draw_tracking(frame, filtered_points, avg_x, avg_y)
                draw_bounding_box(frame, bbox)
                points = add_more_interesting_points(gray_frame, bbox, filtered_points, FEATURE_PARAMS)
                points, _, _ = filter_points(points)  # Ensure points are filtered before updating
                points = points.reshape(-1, 1, 2)
                old_gray = gray_frame
            else:
                face_found = False

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    reference_encoding = load_reference_encoding(REFERENCE_IMAGE_PATH)
    follow_face(VIDEO_PATH, reference_encoding)
