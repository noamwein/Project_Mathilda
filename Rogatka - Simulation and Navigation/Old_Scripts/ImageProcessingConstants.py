import cv2

# CONSTANTS
START_FROM_SECONDS = 0  # Video start time in seconds
PROCESS_EVERY_FRAMES = 5  # Process face recognition every X frames
MIN_TRACKING_POINTS = 4
DISTANCE_THRESHOLD = 150
PADDING_PIXELS = 0
INITIAL_BBOX_SIZE = 150  # Initial bounding box size around detected face
ORIGINAL_CAM_WIDTH = 3840  # original frame width (camera properties)
ORIGINAL_CAM_HEIGHT = 2160  # original frame height (camera properties)
FRAME_WIDTH = 800  # Display frame width (resizing it to a small window)
FRAME_HEIGHT = 450  # Display frame height (resizing it to a small window)

# Paths
PATH = r"media"
VIDEO_PATH = PATH + r"/beker_from_drone.MP4"
VIDEO_PATH = PATH + r"/beker_from_drone.MP4"
REFERENCE_IMAGE_PATH = PATH + r"/b6.png"

# Parameters for Shi-Tomasi corner detection
FEATURE_PARAMS = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

# Parameters for Lucas-Kanade optical flow
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
