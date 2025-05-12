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

# Drone Algorithm
START_LAT = 31.76953 # Southwest corner of the search area.
START_LON = 35.19831 # Southwest corner of the search area.
LONG_SEGMENT = 3
SHORT_SEGMENT = 2
TURN_ANGLE = 90
STEPS = 3  # number of snake segments
INITIAL_ANGLE = 0
RIGHT_ANGLE = (INITIAL_ANGLE + TURN_ANGLE) % 360
LEFT_ANGLE = (INITIAL_ANGLE - TURN_ANGLE) % 360

# Drone Client
MAXIMUM_DISTANCE = 12
KILL_SWITCH_CHANNEL = '8'
KILL_SWITCH_MODE = 'ALTHOLD'

PIXEL_THRESHOLD = 10
YAW_PIXEL_THRESHOLD = 400
YAW_FACTOR = 0.005
SPEED_FACTOR = 0.0015
ANGLE_TOLERANCE = 200
ERROR_TOLERANCE_RADIUS = 200
MAX_SPEED = 1

# Servo
SERVO_PIN = 3
COOLDOWN_TIME=2 #time between drops

# Utils
METERS_PER_DEGREE = 111319.5  # Approx. meters per one degree latitude

