<iframe width="560" height="315" src="https://www.youtube.com/embed/Ru6Z45Rh16o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# LightHouse
### Interactive computer vision art installation
A cross between a watchtower and a lighthouse, switching between shining a guiding light and compulsively tracking passersby.
The tower is a tall, thin, triangular pyramid, each side of which is a screen which has images projected onto it.

### Components
1. Control and motors
A Raspberry pi connected to a servo motor controlling the tilt, a stepper motor + driver controlling the pan movement, a camera and a flashlight.
The Raspberry pi must run the Pigpio daemon that enables controling it's i/o pins remotely and send the video over UDP by some software (such as VLC).

2. Projection mapping
The software runs on the same computer as the main control code, and is connected to 3 projectors. it receives instructions from the main control code and changes the image the projectors would project on the tower. The software must have OSC communication enabled and set up with the same addresses as specified in the config file.

3. Main control code
This program receives images from the Raspberry pi and according to the images run one of two modes: DETECTION: *tell projection software (over OSC protocol message) to project lighthouse image on the tower. *move the camera/light on a constant speed to the right. *if a person found, save their location and switch to TRACKING mode TRACKING: *tell projection software to project eyes on the tower. *move camera/light in a way that the person would be in the center of the image. *if person lost/freeze occured, change to DETECTION mode.

Each of this program's features (pan movement, tilt movement, receiving images, detection/tracking, OSC communication) runs on a different thread.

### Files
FILES: Main.py - the main part of the program. config.py - a file with constants that can be tweeked to change the behaviour of the installation. util.py - a file containing static functions

two files pertaining to the object recognition model: MobileNetSSD_deploy.prototxt MobileNetSSD_deploy.caffemodel

video files: lighthouse_pannel_*_animation.mp4 - an animation representing a lighthouse for each pannel of the tower watch_tower_animation.mp4 - an animation representing a watchtower

### Requirements

Required python libraries: cv2, imutils, numpy, threading, pythonosc, pigpio, socket, time, collections, copy.

### Created by

Shahar Cohen and Dalik Samkai.

Shahar's info is on GitHub (link on the top left). Hire Him!
