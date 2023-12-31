# This script shows how to open a camera the picamera2 module and grab frames and show these.
# Kim S. Pedersen, 2023

import cv2 # Import the OpenCV library
import time
from pprint import *

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)




print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
imageSize = (854, 480)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

pprint(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(1)  # wait for camera to setup


# Open a window
WIN_RF = "Example 1"
""" cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100) """

cnt = 0

while(True):
    input("Press Enter to capture image")
    image = cam.capture_array("main")
    cnt += 1

    # save the image to disk
    cv2.imwrite("test" + str(cnt) + ".jpg", image)
    

# Finished successfully