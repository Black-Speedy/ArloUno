from robot_driving_states import RobotController
from rrt import *
import sys
import os
sys.path.append(("Self-localization"))
import selflocalize
import camera
import time
import numpy as np
import robot_driving_states
import math

robot_x = 0
robot_y = 0
robot_theta = 0


landmarkIDs = [8, 2, 50, 51]
landmarks = {
    8: (0.0, 0.0),  # Coordinates for L1
    2: (0.0, 300.0),  # Cordinates for L2
    50: (400.0, 0.0), # Cordinates for L3
    51: (400.0, 300.0) # Cordinates for L4
}

landmark_dists = {
    8: -1,
    2: -1,
    50: -1,
    51: -1
}


def main():
    cam = camera.Camera(0, 'arlo', useCaptureThread=True)

    r = RobotController([], 0, 0, 0, FollowRRT=False)

    # Find robot position
    foundPos = False
    theta_turned = 0.0

    landmarks_found = []

    ctime = time.perf_counter()

    while (not foundPos):
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()

        if r.ds == robot_driving_states.DriveState.TURN:
            continue

        ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())

        if ids is not None:
            for i in range(0, len(ids)):
                if ids[i] == 51:
                    continue
                if ids[i] in landmarkIDs:
                    if ids[i] not in landmarks_found:
                        landmarks_found.append(ids[i])
                    print(f"ids: {ids}, dists: {dists}, angles: {angles}")
                    landmark_dists[ids[i]] = (dists[i])

        if (len(landmarks_found) == 2):
            foundPos = True
        else:
            if r.stopTimer < time.perf_counter():
                # rotate slightly
                r.turnDegree(15, "left")
                r.ds = robot_driving_states.DriveState.TURN
                theta_turned += 15

    # Calculate robot position
    A_id = landmarks_found[0]
    B_id = landmarks_found[1]
    distance_to_A = landmark_dists[A_id] + 22.5  # Distance to Landmark A
    distance_to_B = landmark_dists[B_id] + 22.5  # Distance to Landmark B
    # Distance between Landmark A and B
    dAB = np.sqrt((landmarks[A_id][0] - landmarks[B_id][0])
                  ** 2 + (landmarks[A_id][1] - landmarks[B_id][1])**2)
    print(dAB)

    # Calculate robot's position
    cos_theta = (distance_to_A**2 - distance_to_B **
                 2 + dAB**2) / (2 * dAB * distance_to_A)
    print(f"before cos theta: {cos_theta}")

    theta = math.acos(cos_theta)

    y = distance_to_A * math.cos(theta)
    x = distance_to_A * math.sin(theta)
    pos_toL2vec = np.abs(np.array([x-r.x, landmarks[landmarks_found[0]][1]-y]))
    print(f"position vector: {pos_vec}")
    L1L2vec = np.array([landmarks[landmarks_found[0]][0], landmarks[landmarks_found[0]][1]])
    
    #Calculate angle between L1 and position vector
    angle = np.rad2deg(np.arccos(np.dot(pos_toL2vec, L1L2vec)/(np.linalg.norm(pos_toL2vec)*np.linalg.norm(L1L2vec))))
    print(f"Pos to L1 angle: {angle}")
    

    if landmarks_found[1] == landmarkIDs[1]:  # We found L1 then L2
        if theta_turned < 180:
            x = -x

    elif landmarks_found[1] == landmarkIDs[2]:  # We found L1 then L3
        if theta_turned > 180:
            tmp = x
            x = y
            y = -tmp

    print(f"theta turned: {theta_turned}")
    print(
        f"distance to a: {distance_to_A}, distance to b: {distance_to_B}, theta: {theta}\n robot pose x: {x}, y: {y}")

    # theta need to be adjusted, as we use the angle from the first point when we see the landmark.
    pos = (x, y, np.arctan2(
        (landmarks[landmarks_found[1]][1] - y), (landmarks[landmarks_found[1]][0] - x)))
    print(f"robots theta: {pos[2]}, in deg {np.rad2deg(pos[2])}")

    r.x = pos[0]
    r.y = pos[1]
    r.theta = pos[2]

    # turn back theta turned:
    r.turnDegree(theta_turned, "right")
    r.ds = robot_driving_states.DriveState.TURN

    while (r.ds == robot_driving_states.DriveState.TURN):
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()

    # Take a picture and look for L1

    tries = 0
    while tries < 20:
        ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())
        if ids is not None:
            for i in range(0, len(ids)):
                if ids[i] == landmarkIDs[0]:
                    # First rotate towards it
                    print(f"ids: {ids}, dists: {dists}, angles: {angles}")
                    if angles[i] > 0:
                        r.turnDegree(np.rad2deg(-angles[i]), "right")
                        r.ds = robot_driving_states.DriveState.TURN
                    else:
                        r.turnDegree(np.rad2deg(angles[i]), "left")
                        r.ds = robot_driving_states.DriveState.TURN

                    while (r.ds == robot_driving_states.DriveState.TURN):
                        if ctime + 0.001 < time.perf_counter():
                            print("turning")
                            r.update()
                            ctime = time.perf_counter()

                    # Drive towards it
                    r.straight64(dists[i] - 20)
                    r.ds = robot_driving_states.DriveState.STRAIGHT

                    while (r.ds == robot_driving_states.DriveState.STRAIGHT):
                        if ctime + 0.001 < time.perf_counter():
                            r.update()
                            ctime = time.perf_counter()
                        
                    r.r.stop()
                    tries = 30
                    break

        else:
            tries += 1
            print("Could not find L1 in this pic")

    print(f"robot pose: x: {r.x}, y: {r.y}, theta: {np.rad2deg(r.theta)}")

    # locate next landmark
    theta_to_add = 0
    if r.theta > 0:
        theta_to_add = r.theta
    else:
        theta_to_add: -r.theta

    arc = np.arctan2(landmarks[landmarkIDs[1]][1] -
                     r.y, landmarks[landmarkIDs[1]][0] - r.x)
    print(f"arc: {arc}")
    degrees_to_turn = theta_to_add + arc
    print(f"degrees to turn: {np.rad2deg(degrees_to_turn)}")




if __name__ == '__main__':
    main()
