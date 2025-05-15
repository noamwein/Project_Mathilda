# General Setup
INITIAL_ALTITUDE = 6

# RPI Related
THRESHOLD_TEMPERATURE = 75

# Image Detection
## Color
FRAME_WIDTH = 640  # Display frame width (resized window)
FRAME_HEIGHT = 480  # Display frame height (resized window)
CENTERED_X = 1080 // 2  # x's pixel of the dropped object
CENTERED_Y = 1500  # y's pixel of the dropped object
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

# Drone Algorithm
START_LAT = 31.76957 # Southwest corner of the search area.
START_LON = 35.19831 # Southwest corner of the search area.
LONG_SEGMENT = 4
SHORT_SEGMENT = 3
TURN_ANGLE = 90
STEPS = 2  # number of snake segments
INITIAL_ANGLE = 0
RIGHT_ANGLE = (INITIAL_ANGLE + TURN_ANGLE) % 360
LEFT_ANGLE = (INITIAL_ANGLE - TURN_ANGLE) % 360
PIXELS_PER_RAD = 2000

RE_SEARCH_LIMIT = 10

# Drone Client
MAXIMUM_DISTANCE = 12
KILL_SWITCH_CHANNEL = '8'
KILL_SWITCH_MODE = 'ALTHOLD'

YAW_TOLERANCE_RADIUS = 300
YAW_TOLERANCE_THRESHOLD = 200
DROP_RADIUS = 150
MAX_SPEED = 1

# PID gains
KP_V = 0.00075
KI_V = 0.0
KD_V = -0.0002

KP_YAW = 0.0175
KI_YAW = 0.0
KD_YAW = 0.0

# Miss threshold for resetting stale PID
MISS_LIMIT = 0
YAW_INTEGRAL_MAX = 10
VEL_INTEGRAL_MAX = 10

# Servo
SERVO_PIN = 11
COOLDOWN_TIME = 3 #time between drops

# Utils
METERS_PER_DEGREE = 111319.5  # Approx. meters per one degree latitude

# Servo
CLOSE_ANGLE = 0
OPEN_ANGLE = 160
SERVO_ANGLES = [115, 125, OPEN_ANGLE]
