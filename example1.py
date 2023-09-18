# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2  # Import the OpenCV library


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

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
print("now looking for markers")

while cv2.waitKey(4) == -1:  # Wait for a key pressed event
    retval, frameReference = cam.read()  # Read frame

    if not retval:  # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)

    # Show frames

    # Use openCV ArUco library to detect markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frameReference, arucoDict, parameters=arucoParams)

    if (len(corners) > 0):
        print(cv2.aruco.estimatePoseSingleMarkers(corners, markerLength=14.6))
        



    # Show the frame
    #cv2.imshow(WIN_RF, frameReference)



# Finished successfully
