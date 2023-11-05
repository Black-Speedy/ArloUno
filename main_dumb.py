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


landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for L1
    2: (0.0, 300.0),  # Cordinates for L2
    3: (400.0, 0.0), # Cordinates for L3
    4: (400.0, 300.0) # Cordinates for L4
}

landmark_dists = {
    1: -1,
    2: -1,
    3: -1,
    4: -1
}

def Turn_Robot(r, theta, dir):
    if dir == "right":
        r.turnDegree(np.rad2deg(theta), "right")
    else:
        r.turnDegree(np.rad2deg(theta), "left")

    r.ds = robot_driving_states.DriveState.TURN

    ctime = time.perf_counter()
    while (r.ds == robot_driving_states.DriveState.TURN):
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()

    r.r.stop()

def Drive_Robot(r, dist):
    r.straight64(dist)
    r.ds = robot_driving_states.DriveState.STRAIGHT

    ctime = time.perf_counter()
    while (r.ds == robot_driving_states.DriveState.STRAIGHT):
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()

    r.r.stop()


def main():
    cam = camera.Camera(0, 'arlo', useCaptureThread=True)

    r = RobotController([], 0, 0, 0, FollowRRT=False)

    """ for i in range(0, 4):
        Turn_Robot(r, np.deg2rad(90), "left")
        while r.stopTimer > time.perf_counter():
            r.update()
    r.r.stop()
    exit() """

    """ print("Starting robot")
    print("Robot started")
    print("Robot state set to straight")
    ctime = time.perf_counter()
    r.ds = robot_driving_states.DriveState.STRAIGHT
    print(f"ctime: {ctime}")
    r.straight64(100)
    while (r.ds == robot_driving_states.DriveState.STRAIGHT):
        if ctime + 0.001 < time.perf_counter():
            print(f"robot state: {r.ds}")
            r.update()
            ctime = time.perf_counter()
    print("Robot stopped")
    r.r.stop()
    exit() """

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


    current_goal = 0
    turnTries = 0

    while current_goal < 5:
        tries = 0
        while tries < 20:
            ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())
            if ids is not None:
                for i in range(0, len(ids)):
                    if ids[i] == landmarkIDs[current_goal]:
                        # First rotate towards it
                        print(f"ids: {ids}, dists: {dists}, angles: {angles}")

                        if angles[i] > 0:
                            r.turnDegree(np.rad2deg(angles[i]), "left")
                            r.ds = robot_driving_states.DriveState.TURN
                        else:
                            r.turnDegree(np.rad2deg(-angles[i]), "right")
                            r.ds = robot_driving_states.DriveState.TURN

                        while (r.ds == robot_driving_states.DriveState.TURN):
                            if ctime + 0.001 < time.perf_counter():
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
                        print(f"Robot pose: x: {r.x}, y: {r.y}, theta{np.rad2deg(r.theta)}")
                        tries = 30
                        current_goal += 1
                        break

            else:
                tries += 1

        if tries == 20:
            Turn_Robot(r, np.deg2rad(15), "left")

            if turnTries < 24: 
                turnTries += 1
                continue
            else:
                print(f"Could not find L{current_goal + 1}")

        print(f"robot pose: x: {r.x}, y: {r.y}, theta: {np.rad2deg(r.theta)}")

        # locate next landmark
        arc = np.arctan2(landmarks[landmarkIDs[current_goal]][1] -
                         r.y, landmarks[landmarkIDs[current_goal]][0] - r.x)
        print(f"arc: {np.rad2deg(arc)}")
        degrees_to_turn = arc - r.theta

        if degrees_to_turn > 2*np.pi:
            print(f"Degrees too large!!! {np.rad2deg(degrees_to_turn)}")
            degrees_to_turn = degrees_to_turn % (np.pi * 2)

        if np.rad2deg(degrees_to_turn) > 180:
            degrees_to_turn = 2* np.pi - degrees_to_turn
        elif np.rad2deg(degrees_to_turn) < -180:
            degrees_to_turn = 2 * np.pi + degrees_to_turn


        print(f"degrees to turn: {np.rad2deg(degrees_to_turn)}")

        r.ds=robot_driving_states.DriveState.TURN
        if degrees_to_turn > 0:
            r.turnDegree(np.rad2deg(degrees_to_turn), "left")
        else:
            r.turnDegree(np.rad2deg(-degrees_to_turn), "right")
            
        while (r.ds == robot_driving_states.DriveState.TURN):
                            if ctime + 0.001 < time.perf_counter():
                                r.update()
                                ctime = time.perf_counter()

        r.r.stop()

if __name__ == '__main__':
    main()
