# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2  # Import the OpenCV library
import numpy as np
import time

def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )


print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)


if not cam.isOpened():  # Error
    print("Could not open camera")
    exit(-1)

# Open a window
WIN_RF = "Example 1"
""" cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100) """

focal_length = 1136
angle_error = 11


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
cameraMatrix = np.array([[focal_length, 0, 640],[0, focal_length, 360],[0, 0, 1]])
print("now looking for markers")

def lookBox():
    retval, frameReference = cam.read()  # Read frame

    if not retval:  # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)

    # Use openCV ArUco library to detect markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frameReference, arucoDict, parameters=arucoParams)

    rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
        corners, markerLength=0.146, cameraMatrix=cameraMatrix, distCoeffs=None)
    
    #cv2.aruco.estimatePoseSingleMarkers returnerer translation vector(tvec) og rotation vector(rvec)

    distance = None
    boxDegrees = None
    Xdegrees = None
    if (len(corners) > 0):
        for id in ids:
            print(id)
        radians = tvecs[0][0][0]
        degrees = np.degrees(radians)+ angle_error
        Xdegrees = np.degrees(rvecs[0][0][2])
        boxDegrees = np.degrees(radians)+ angle_error
        distance = tvecs[0][0][2]
        # Print tvecs in format: distance, height, angle in degrees
        print(f"distance = {distance}")
        print(f"X = {Xdegrees}")
        print(f"{boxDegrees}")
        print()
        return distance, boxDegrees, Xdegrees
    else:
        return distance, boxDegrees, Xdegrees



