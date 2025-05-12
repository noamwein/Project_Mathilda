# General Setup
INITIAL_ALTITUDE = 6

# RPI Related
THRESHOLD_TEMPERATURE = 75

# Image Detection
## Color
FRAME_WIDTH = 640  # Display frame width (resized window)
FRAME_HEIGHT = 480  # Display frame height (resized window)
CENTERED_X = 1080 // 2  # x's pixel of the dropped object
CENTERED_Y = 1400  # y's pixel of the dropped object
## Human
MAX_BBOX_WIDTH = 1000
MAX_BBOX_HEIGHT = 1000
DELAY_IN_DETECTION = 120

# RPI Camera
TARGET_BRIGHTNESS = 170
EXPOSURE_TIME = 10
DEFAULT_GAIN = 30
LOW_RESOLUTION = (1280, 720)
HIGH_RESOLUTION = (1920, 1080)
RESOLUTION = HIGH_RESOLUTION