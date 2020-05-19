import numpy as np

# ********** lighting Constants **********
LAMP_CTRL_PIN = 5
LAMP_MODE = 0  # 0 is ON 1 is OFF

# ********** Communication Constants **********
HEAD_IP = '10.0.0.1'
HEAD_VIDEO_PORT = 8081
HEAD_PIGPIO_PORT = 8888
PROJECTION_PORT = 8010
OSC_TRACKING_ADDR = "/lighthouse/eyes"
OSC_DETECTING_ADDR = "/lighthouse/light"
TRACKING_SIGNAL = 0.9
DETECTING_SIGNAL = 0.1

# ********** OPENCV FILES (should be in the same directory) **********
NET = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"

# ********** computer vision constants and magic numbers **********
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]
BLANK_IMG = np.zeros([100, 100, 3], dtype=np.uint8)
WIDTH = 2
HEIGHT = 3
RED = (0, 0, 255)
INITIAL_BOX = (0, 0, 0, 0)
CENTER_W = 320
CENTER_H = 240
MIN_CONFIDENCE = 0.9
INV_GAMMA = 0.2

# ********** Operational constants and Magic Numbers **********
HEADLESS = False  # false == display image and rectangle, True = no screen used
DETECTING = 0
TRACKING = 1
IDLE = 2
MAX_LOST_TIME = 5  # max number of seconds of losing the object before going back to detection
TIME_IDLE_MULTIPLIER = 0.5  # multiplier to set how long should the system be in idle mode in proportion to
# the previous tracking time.

# ********** Motion Constants **********
#servo motor
TILT_PIN = 17
MIN_PULSE_WIDTH = 2000
MAX_PULSE_WIDTH = 2270
SERVO_INCREMENT_SIZE = 2
MIN_TILT_DIFF = 10 * SERVO_INCREMENT_SIZE

# stepper controlled with direction/pulse driver - DRV8825:
PAN_DIR_PIN = 21
PAN_PULSE_PIN = 20
CLOCKWISE = 1
C_CLOCKWISE = 0
INCREMENT_STEPPER = 4
MIN_PAN_DIFF = 10 * INCREMENT_STEPPER
DETECTION_PPS = 500
TRACKING_BASE_PPS = 500
STEP_ANGLE = 0.9
MICROSTEP_AMOUNT = 32
STEPS_PER_REV = (360 / STEP_ANGLE)*MICROSTEP_AMOUNT
STEPPER_DELAY = 1/STEPS_PER_REV

# ********** time Constants **********
MAX_FREEZE_TIME = 2