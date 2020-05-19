# import constants stored in config file
import config as c
# import static functions stored in util file
import util

import cv2
import imutils
import numpy as np
import threading
from pythonosc import udp_client
import pigpio
import socket
import time
from collections import  deque
from copy import copy

# ===========Short script to get this machines IP============
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
MY_IP = str(s.getsockname()[0])
s.close()
# ===========================================================


class LightHouse:
    """
    this class controls an art installation of a tower/lighthouse that tracks people and shines a light on them.
    the tower is a tall, thin, triangular pyramid, each side of which is a screen which has images projected onto it.

    watch it operate here: https://youtu.be/Ru6Z45Rh16o

    the system comprised of 3 main parts:
    1) raspberry pi (controlls a camera and flashlight mounted on pan/tilt motors)
        the raspberry pi sends this class the camera footage (it should run a video over udp software, such as vlc.)
        the raspi also controls the pan/tilt motors according to instructions sent by this class.
        (it should run a pigpio daemon to receive and perform the commands)

    2) a projection mapping software connected to 3 projectors
        the software runs on the same computer as this program. it receives instructions from this program
        and changes the image the projectors would project on the tower.

    3) this program
        this class receives images from the raspi and according to the images run one of two modes:
        DETECTION:
            *tell projection software to project lighthouse on the tower.
            *move the camera/light on a constant speed to the right.
            *if a person found, save their location and switch to TRACKING mode
        TRACKING:
            *tell projection software to project eyes on the tower.
            *move camera/light in a way that the person would be in the center of the image.
            *if person lost/freeze occured, change to DETECTION mode.

    this class runs each part of it's responsibilities as a separate thread.
    """
    def __init__(self):
        # ********** Operational Vars **********
        self.is_operational = True
        self.mode = c.DETECTING
        self.tracker_init_needed = True

        # ********** CV vars **********

        # init caffe net:
        self.net = cv2.dnn.readNetFromCaffe(c.NET, c.MODEL)

        # init video feed from raspberry pi:
        self.videoFeed = cv2.VideoCapture("http://{}:{}/".format(c.HEAD_IP, c.HEAD_VIDEO_PORT))

        # ********** frame vars **********
        self.frame = c.BLANK_IMG
        self.gray_frame = c.BLANK_IMG
        self.output_frame = c.BLANK_IMG

        # ********** tracked/detected coordinates **********
        self.bodies = list()
        self.detected_box = c.INITIAL_BOX
        self.tracked_box = c.INITIAL_BOX
        self.firstPoint = (0, 0)
        self.secondPoint = (0, 0)
        self.box_center = (0, 0)

        # ********** time keeper vars **********
        self.event_queue = deque()
        self.object_lost_timer = 0
        self.tracking_timer = 0
        self.freeze_time = 0
        self.soft_freeze_time = 0
        self.idle_begin_time = 0
        self.idle_end_time = 0

        # ********** remote raspberry pi control vars **********
        self.pi = pigpio.pi(c.HEAD_IP)  # set pi as the raspberry pi with ip address HEAD_IP
        self.pi.set_mode(c.LAMP_CTRL_PIN, pigpio.OUTPUT)  # set lamp control pin as output
        self.pi.write(c.LAMP_CTRL_PIN, c.LAMP_MODE)  # turn lamp on or off (depending on mode in config file)

        # ********** movement vars **********
        self.needed_pan_movement = 0
        self.needed_tilt_movement = 0

    def run(self):
        """ define and run threads"""
        # def threads:
        img_process_thread = threading.Thread(target=self.process_frame, daemon=True)
        tracker_thread = threading.Thread(target=self.track, daemon=True)
        tilt_control_thread = threading.Thread(target=self.control_tilt, daemon=True)
        pan_control_thread = threading.Thread(target=self.control_pan, daemon=True)
        time_keeper_thread = threading.Thread(target=self.time_keeper, daemon=True)
        osc_movement_thread = threading.Thread(target=self.osc_projection_client, daemon=True)

        # Start threads:

        if not c.HEADLESS:  # if display exists, show frames and detection
            show_frame_thread = threading.Thread(target=self.show_frame, daemon=True)
            show_frame_thread.start()

        img_process_thread.start()

        time.sleep(3)  # allow time of video processing to initialize

        tracker_thread.start()
        tilt_control_thread.start()
        pan_control_thread.start()
        time_keeper_thread.start()
        osc_movement_thread.start()

        while self.is_operational:  # run until issue arrises
            time.sleep(0.035)
            pass

        self.handle_exit()

    def show_frame(self):
        """
        output to screen the image received by the camera with tracked object in red rectangle (if tracking)
        """
        while self.is_operational:
            self.output_frame = self.frame

            if self.mode == c.TRACKING:
                # draw rectangle according to box
                cv2.rectangle(self.output_frame, self.firstPoint, self.secondPoint, c.RED, c.WIDTH)
                # draw dot in center of rect:
                cv2.line(self.output_frame, self.box_center, self.box_center, c.RED, c.WIDTH)

            enlarged_image = imutils.resize(self.output_frame, width=1920, height=1440)

            cv2.imshow("preview", enlarged_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                self.handle_exit()

    def update_box(self, input_box=(0, 0, 0, 0)):
        """
        update variables relating to tracked object coordinates and needed movement according to
        input_box (the tracked object's new location)
        :param input_box:
        """
        # Get 2 opposite points from box:
        self.firstPoint = (int(input_box[0]), int(input_box[1]))
        self.secondPoint = (int(input_box[0] + input_box[c.WIDTH]), int(input_box[1] + input_box[c.HEIGHT]))
        self.box_center = (int((input_box[0] + 0.5 * input_box[c.WIDTH])), int((input_box[1] + 0.5 * input_box[c.HEIGHT])))

        self.needed_pan_movement = util.required_movement_to_re_center_tracking_box(self.box_center[0])
        self.needed_tilt_movement = util.required_movement_to_re_center_tracking_box(self.box_center[1], 1)

    def process_frame(self):
        """
        correct frame from camera and store in BW and color versions.
        """
        while self.is_operational:

            # Reading the each frame of the video
            ok, frame = self.videoFeed.read()

            # Check if extracting frame worked
            if not ok:
                print("could not obtain frame")
                self.videoFeed.release()
                exit()

            # rotate frame:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # gamma correction:
            table = np.empty((1, 256), np.uint8)
            for i in range(256):
                table[0, i] = np.clip(pow(i/255.0, c.INV_GAMMA)* 255.0, 0, 255)

            frame = cv2.LUT(frame, table)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            self.frame = frame
            self.gray_frame = gray

            time.sleep(0.0005)

    def detect(self):
        """
        attempt to detect a person in the current frame.
        if detected, update mode to tracking, update tracking box and tracking timer.
        """
        frame = self.frame

        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843,
                                     (300, 300), 127.5)

        # pass the blob through the network and obtain the detections and predictions
        self.net.setInput(blob)
        detections = self.net.forward()

        max_confidence = c.MIN_CONFIDENCE
        box = c.INITIAL_BOX

        # loop over the detections:
        for i in range(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with the prediction
            confidence = detections[0, 0, i, 2]
            if confidence <= max_confidence:
                continue

            max_confidence = confidence


            # net 2:
            idx = int(detections[0, 0, i, 1])
            label = c.CLASSES[idx]

            if label != "person":
                continue

            # compute the (x, y)-coordinates of the bounding box for the object
            new_box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = new_box.astype("int")

            box = (startX, startY, endX - startX, endY - startY)

        if box != c.INITIAL_BOX:
            #print("tracking") # uncomment for feedback
            self.detected_box = box
            self.update_box(box)
            self.mode = c.TRACKING
            self.tracker_init_needed = True
            self.tracking_timer = time.time()

    def track(self):
        """
        update tracking box to new location of tracked object.
        if object lost for a significant amount of time, update mode to detecting.
        """
        while self.is_operational:

            if self.mode == c.DETECTING:
                self.detect()
                time.sleep(0.035)
                continue

            if self.mode == c.IDLE:

                if time.time() > self.idle_end_time:
                    # zero vars and switch to DETECTING
                    self.mode = c.DETECTING
                    self.idle_end_time = 0

                    # print("detecting")  # uncomment for feedback

                time.sleep(0.03)
                continue

            # alt - global frames:
            frame = self.frame

            if self.tracker_init_needed:  # init tracker:
                tracker = cv2.TrackerCSRT_create()  # optionally: TrackerKCF_create()
                res = tracker.init(frame, self.detected_box)

                if not res:
                    continue

                self.tracker_init_needed = False
                self.detected_box = c.INITIAL_BOX


            ok, box = tracker.update(frame)

            # if no box was found, check if idle time too long
            if not ok or self.freeze_has_occurred():
                # if idle time wasn't measured yet, start measuring:
                if self.object_lost_timer == 0:
                    self.object_lost_timer = time.time()

                # if idle time is too long, switch to IDLE
                lost_time = time.time() - self.object_lost_timer

                if lost_time > c.MAX_LOST_TIME or self.freeze_has_occurred():
                    # zero and init variables and switch to IDLE

                    self.idle_end_time = time.time() + (time.time() - self.tracking_timer) * c.TIME_IDLE_MULTIPLIER

                    self.object_lost_timer = 0
                    self.tracking_timer = 0

                    self.mode = c.IDLE
                    self.tracker_init_needed = True
                    print("idle")

            else:
                # Update lighthouse's ROI box
                self.update_box((int(box[0]), int(box[1]), int(box[2]), int(box[3])))

    def control_tilt(self):
        """
            controls the tilt motion done by a servo motor controlled by pigpio daemon that
            runs on the raspberry pi.
        """

        self.pi.set_mode(c.TILT_PIN, pigpio.OUTPUT)

        width = c.MIN_PULSE_WIDTH  # the minimum pulse width for my set up is the tallest tilt setting.

        self.pi.set_servo_pulsewidth(c.TILT_PIN, width)

        while self.is_operational:
            if self.mode != c.TRACKING:
                width = c.MIN_PULSE_WIDTH
                self.pi.set_servo_pulsewidth(c.TILT_PIN, width)

            if self.mode == c.TRACKING:

                if self.needed_tilt_movement != 0:

                    intensity = abs(self.needed_tilt_movement)  # absolute size of difference

                    direction = intensity / self.needed_tilt_movement  # direction of difference

                    width += c.SERVO_INCREMENT_SIZE * direction * int(2.5 ** intensity)

                    width = util.clip_servo_pulse_width(width)

                    self.pi.set_servo_pulsewidth(c.TILT_PIN, width)

            time.sleep(0.05)

    def control_pan(self):
        """
        controls the pan motion done by a stepper motor connected to the driver, controlled by pigpio daemon that
        runs on the raspberry pi.
        """
        self.pi.set_mode(c.PAN_DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(c.PAN_PULSE_PIN, pigpio.OUTPUT)

        detecting_mode_freq_set = False
        self.pi.write(c.PAN_DIR_PIN, 1)
        while self.is_operational:
            if self.mode == c.DETECTING:

                if not detecting_mode_freq_set:
                    self.pi.write(c.PAN_DIR_PIN, 1)  # set direction to right

                    # change pwm frequency to the detection mode values:
                    self.pi.set_PWM_dutycycle(c.PAN_PULSE_PIN, 128)  # PWM 1/2 On 1/2 Off
                    self.pi.set_PWM_frequency(c.PAN_PULSE_PIN, c.DETECTION_PPS)
                    detecting_mode_freq_set = True

            else:  # (if mode is tracking)
                detecting_mode_freq_set = False

                if self.needed_pan_movement == 0:
                    self.pi.write(c.PAN_PULSE_PIN, 0)
                    time.sleep(c.STEPPER_DELAY)

                else:
                    if self.needed_pan_movement > 0:
                        self.pi.write(c.PAN_DIR_PIN, c.CLOCKWISE)
                    else:
                        self.pi.write(c.PAN_DIR_PIN, c.C_CLOCKWISE)

                    # Set duty cycle and frequency:
                    self.pi.set_PWM_dutycycle(c.PAN_PULSE_PIN, 128)  # PWM 1/2 On 1/2 Off
                    pulse_per_second = min(c.TRACKING_BASE_PPS + 100 * (3 ** (abs(self.needed_pan_movement))), 8000)
                    self.pi.set_PWM_frequency(c.PAN_PULSE_PIN, pulse_per_second)

            time.sleep(0.0015)

    def time_keeper(self):
        """
        this thread keeps valuable timing information such as the time elapsed since tracking started,
        time elaped since freeze started (tracking error)...
        """
        prev_box_center = (0, 0)
        freeze_start = 0
        soft_freeze_start = 0

        while self.is_operational:
            time.sleep(0.005)

            # if not tracking, zero vars and continue loop
            if self.mode != c.TRACKING:
                self.freeze_time = 0
                freeze_start = 0
                self.soft_freeze_time = 0
                soft_freeze_start = 0
                prev_box_center = (0, 0)
                continue

            tuple_diff = util.tuple_diff(prev_box_center, self.box_center)

            # handle soft_freeze
            if tuple_diff < 10 and self.needed_pan_movement == 0:
                if soft_freeze_start == 0:
                    soft_freeze_start = time.perf_counter()

                self.soft_freeze_time = time.perf_counter() - soft_freeze_start

            else:
                self.soft_freeze_time = 0
                soft_freeze_start = 0

            # handle hard freeze:
            if prev_box_center == self.box_center:

                if freeze_start == 0:
                    freeze_start = time.perf_counter()

                self.freeze_time = time.perf_counter() - freeze_start

            # handle movement:
            else:
                if freeze_start != 0:
                    freeze_start = 0
                    self.freeze_time = 0

            # updating prevs:
            prev_box_center = copy(self.box_center)

    def freeze_has_occurred(self):
        """
        returns true if freeze hase occured (happens when tracker thinks it's tracking the corner)
        """
        if self.freeze_time > c.MAX_FREEZE_TIME:
            print("froze")
            return True

        if util.tuple_within_range(self.box_center, (-30, 50), (-30, 50)):
            if self.freeze_time > 0.2 or self.soft_freeze_time > 0.2:
                print("corner freeze")
                return True

        if self.soft_freeze_time > c.MAX_FREEZE_TIME * 3:
            print("soft freeze")
            return True

        return False

    def osc_projection_client(self):
        """
        sends an OSC message to the program in charge of projection mapping, indicating whether
        in tracking or detecting mode.
        """
        projection_client = udp_client.SimpleUDPClient(MY_IP, c.PROJECTION_PORT)
        mode = c.DETECTING

        while self.is_operational:

            if mode == self.mode:
                time.sleep(0.05)
                continue

            mode = self.mode

            if mode == c.TRACKING:
                address = c.OSC_TRACKING_ADDR
                signal = c.TRACKING_SIGNAL
                # print("projected eyes")  # uncomment for feedback

            else:
                address = c.OSC_DETECTING_ADDR
                signal = c.DETECTING_SIGNAL
                # print("projected lighthouse")  # uncomment for feedback

            projection_client.send_message(address, signal)

            time.sleep(0.05)

    def handle_exit(self):
        """
        safely exit program
        """
        self.videoFeed.release()
        cv2.destroyAllWindows()

        try:
            self.pi.write(c.PAN_PULSE_PIN, 0)
            self.pi.stop()

        except AttributeError:
            print("pigpio deamon previously stopped")

        self.is_operational = False
        exit()


if __name__ == "__main__":
    lighthouse = LightHouse()

    lighthouse.run()



