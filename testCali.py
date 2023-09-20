import numpy as np
import cv2
from cv2 import aruco

# Define ArUco dictionary and board parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
board = aruco.GridBoard_create(4, 5, 0.04, 0.01, aruco_dict)

# Lists to store 3D world points and 2D image points
obj_points = []  # 3D world points
img_points = []  # 2D image points

# Capture images and detect markers
cap = cv2.VideoCapture(0)  # Use the appropriate camera index
while True:
    ret, frame = cap.read()
    
    # Detect ArUco markers in the frame
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
    
    if ids is not None:
        # Draw detected markers
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Collect 3D and 2D points
        for i, marker_id in enumerate(ids):
            if marker_id[0] == 0:  # Change this condition as needed
                obj_points.append(board.chessboardCorners[i])
                img_points.append(corners[i])

    cv2.imshow('Frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, frame.shape[::-1], None, None)

# Print calibration results
print("Camera matrix:")
print(mtx)
print("\nDistortion coefficients:")
print(dist)
