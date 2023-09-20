# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2  # Import the OpenCV library
import numpy as np

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
f = 1126

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
cameraMatrix = np.array([[f, 0, 640],[0, f, 360],[0, 0, 1]])
print("now looking for markers")


while cv2.waitKey(4) == -1:  # Wait for a key pressed event
    retval, frameReference = cam.read()  # Read frame

    if not retval:  # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)

    # Use openCV ArUco library to detect markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frameReference, arucoDict, parameters=arucoParams)

    rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
        corners, markerLength=0.146, cameraMatrix=cameraMatrix, distCoeffs=None)
    
    #cv2.aruco.estimatePoseSingleMarkers returnerer translation vector(tvec) og rotation vector(rvec)

    if (len(corners) > 0):
        for id in ids:
            print(id)
        # print tvecs in format: distance, height, angle 
        print(
            #f"distance = {tvecs[0][0][2]}, height = {tvecs[0][0][1]}, angle = {tvecs[0][0][0]}")
            f {rvecs[0][0][2]}, {rvecs[0][0][1]}, {rvecs[0][0][0]})


