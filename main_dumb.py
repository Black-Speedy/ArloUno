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


landmarkIDs = [1, 2, 3, 4, 1]
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

def Turn_To_Landmark(r, landmarkID):
        # locate next landmark
        arc = np.arctan2(landmarks[landmarkIDs[landmarkID]][1] -
                         r.y, landmarks[landmarkIDs[landmarkID]][0] - r.x)
        degrees_to_turn = arc - r.theta

        if np.abs(degrees_to_turn) > 2*np.pi:
            print(f"Degrees too large!!! {np.rad2deg(degrees_to_turn)}")
            degrees_to_turn = degrees_to_turn % (np.pi * 2)

        if np.rad2deg(degrees_to_turn) > 180:
            degrees_to_turn = degrees_to_turn - 2* np.pi
        elif np.rad2deg(degrees_to_turn) < -180:
            degrees_to_turn = 2 * np.pi + degrees_to_turn

        r.ds=robot_driving_states.DriveState.TURN
        if degrees_to_turn > 0:
            r.turnDegree(np.rad2deg(degrees_to_turn) - 15, "left")
        else:
            r.turnDegree(np.rad2deg(-degrees_to_turn) + 15, "right")
            
        ctime = time.perf_counter()
        while (r.ds == robot_driving_states.DriveState.TURN):
                            if ctime + 0.001 < time.perf_counter():
                                r.update()
                                ctime = time.perf_counter()

def main():
    cam = camera.Camera(0, 'arlo', useCaptureThread=True)

    r = RobotController([], 0, 0, 0, FollowRRT=False)

    # Find robot position
    foundPos = False

    theta_turned = 0.0

    last_landmark_found = -1

    while not foundPos:
        if theta_turned >= 1:
            leftBlock, rightBlock, frontBlock = r.get_obstacle_distances()
            print(f"left: {leftBlock}, right: {rightBlock}, front: {frontBlock}")
            if frontBlock > 1300:
                Drive_Robot(r, 100)
                r.stopTimer = time.perf_counter() + 0.8
            elif leftBlock > 1100:
                Turn_Robot(r, np.deg2rad(45), "left")
                r.stopTimer = time.perf_counter() + 0.8
                Drive_Robot(r, 100)
                r.stopTimer = time.perf_counter() + 0.8
            elif rightBlock > 1100:
                # Turn right
                Turn_Robot(r, np.deg2rad(45), "right")
                r.stopTimer = time.perf_counter() + 0.8
                Drive_Robot(r, 100)
                r.stopTimer = time.perf_counter() + 0.8                

        theta_turned = 0.0
        landmarks_found = [-1, -1, -1]

        ctime = time.perf_counter()

        while (theta_turned < 360):
            if ctime + 0.001 < time.perf_counter():
                r.update()
                ctime = time.perf_counter()

            if r.ds == robot_driving_states.DriveState.TURN:
                continue

            ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())

            theta_before_reset = 0

            if ids is not None:
                for i in range(0, len(ids)):
                    if ids[i] == landmarkIDs[3]:
                        continue
                    if ids[i] in landmarkIDs:
                        if ids[i] not in landmarks_found:
                            last_landmark_found = ids[i]

                            landmarks_found[ids[i]-1] = ids[i]
                            theta_before_reset = theta_turned
                            theta_turned = 0.0
                        print(f"ids: {ids}, dists: {dists}, angles: {angles}")
                        landmark_dists[ids[i]] = (dists[i])

            found = 0
            for i in range(0, len(landmarks_found)):
                if landmarks_found[i] != -1:
                    found += 1
                elif i == 0:
                    break
            if (found >= 2):
                theta_turned = theta_before_reset
                foundPos = True

                #############################################3
                A_id = landmarks_found[0]

                B_id = -1
                if  landmarks_found[1] != -1:
                    B_id = landmarks_found[1]
                else:
                    B_id = landmarks_found[2]

                distance_to_A = landmark_dists[A_id] + 22.5  # Distance to Landmark A
                distance_to_B = landmark_dists[B_id] + 22.5  # Distance to Landmark B
                # Distance between Landmark A and B
                dAB = np.sqrt((landmarks[A_id][0] - landmarks[B_id][0])
                            ** 2 + (landmarks[A_id][1] - landmarks[B_id][1])**2)
                dAB = dAB - 26

                # Calculate robot's position
                cos_theta = (distance_to_A**2 - distance_to_B **
                            2 + dAB**2) / (2 * dAB * distance_to_A)
                #print(f"before cos theta: {cos_theta}")
                if cos_theta > 1 or cos_theta < -1:
                    foundPos = False
                #############################################3

                break
            else:
                if r.stopTimer < time.perf_counter():
                    # rotate slightly
                    r.turnDegree(15, "left")
                    r.ds = robot_driving_states.DriveState.TURN
                    theta_turned += 15
                    
    # Calculate robot position
    A_id = landmarks_found[0]

    B_id = -1
    if  landmarks_found[1] != -1:
        B_id = landmarks_found[1]
    else:
        B_id = landmarks_found[2]

    distance_to_A = landmark_dists[A_id] + 22.5  # Distance to Landmark A
    distance_to_B = landmark_dists[B_id] + 22.5  # Distance to Landmark B
    # Distance between Landmark A and B
    dAB = np.sqrt((landmarks[A_id][0] - landmarks[B_id][0])
                  ** 2 + (landmarks[A_id][1] - landmarks[B_id][1])**2)
    dAB = dAB - 26

    # Calculate robot's position
    cos_theta = (distance_to_A**2 - distance_to_B **
                 2 + dAB**2) / (2 * dAB * distance_to_A)
    
    #print(f"before cos theta: {cos_theta}")
    theta = 0

    theta = math.acos(cos_theta)


    y = distance_to_A * math.cos(theta)
    x = distance_to_A * math.sin(theta)
    
    found_id = -1
    for i in range(1, len(landmarks_found)):
        if landmarks_found[i] != -1:
            found_id = landmarks_found[i]
            break

    if last_landmark_found == 1:
        theta_turned = np.pi * 2 - theta_turned 

    if found_id == 2:  # We found L1 then L2
        if theta_turned < 180:
            x = -x

    elif found_id == 3:  # We found L1 then L3
        if theta_turned > 180:
            tmp = x
            x = y
            y = -tmp

    print(f"theta turned: {theta_turned}")
    print(
        f"distance to a: {distance_to_A}, distance to b: {distance_to_B}, theta: {theta}\n robot pose x: {x}, y: {y}")

    # theta need to be adjusted, as we use the angle from the first point when we see the landmark.
    pos = (x, y, np.arctan2(
        (landmarks[landmarks_found[last_landmark_found-1]][1] - y),
        (landmarks[landmarks_found[last_landmark_found-1]][0] - x)))
    

    
    print(f"robots theta: {pos[2]}, in deg {np.rad2deg(pos[2])}")

    r.x = pos[0]
    r.y = pos[1]
    r.theta = pos[2]

    # turn back theta turned:
    if theta_turned > 180:
        theta_turned = 360 - theta_turned
        r.turnDegree(theta_turned, "left")
    else:
        r.turnDegree(theta_turned, "right")
        
    r.ds = robot_driving_states.DriveState.TURN

    while (r.ds == robot_driving_states.DriveState.TURN):
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()

    # Take picture and measure landmarks
    current_goal = 0
    turnTries = 0
    while current_goal < 5:
        tries = 0
        while tries < 6:
            ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())
            if tries == 0:
                tries += 1
                continue
            if ids is not None:
                dists_sum = np.zeros(4)
                angles_sum = np.zeros(4)
                times_seen = np.zeros(4)
                for i in range(0, len(ids)):
                    if ids[i] > 4:
                        continue
                    dists_sum[ids[i] - 1] += dists[i]
                    angles_sum[ids[i] - 1] += angles[i]
                    times_seen[ids[i] - 1] += 1

                for i in range(0, len(ids)):
                    if ids[i] > 4:
                        continue
                    dists[i] = dists_sum[ids[i]-1] / times_seen[ids[i]-1]
                    angles[i] = angles_sum[ids[i]-1] / times_seen[ids[i]-1]
                
                # Check if we see the current goal
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
                        r.straight64(dists[i] - 8)
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
                tries += 1
            else:
                tries += 1
        if tries == 6:
            Turn_Robot(r, np.deg2rad(15), "left")

            if turnTries < 24: 
                turnTries += 1
                continue
            else:
                # Drive 1 meter diagonally to the left, and try again
                leftBlock, rightBlock, frontBlock = r.get_obstacle_distances()

                if leftBlock < 130:
                    if rightBlock < 130:
                        # drive 40 cm
                        Drive_Robot(r, 40)
                    # Turn right
                    Turn_Robot(r, np.deg2rad(45), "right")
                    r.stopTimer = time.perf_counter() + 0.8
                    Drive_Robot(r, 100)
                    r.stopTimer = time.perf_counter() + 0.8
                    # degrees to turn
                    Turn_To_Landmark(r, current_goal)
                else:
                    Turn_Robot(r, np.deg2rad(45), "left")
                    r.stopTimer = time.perf_counter() + 0.8
                    Drive_Robot(r, 100)
                    r.stopTimer = time.perf_counter() + 0.8
                    # degrees to turn
                    Turn_To_Landmark(r, current_goal)

                turnTries = 0


        print(f"robot pose: x: {r.x}, y: {r.y}, theta: {np.rad2deg(r.theta)}")

        Turn_To_Landmark(r, current_goal)

        print(f"Our current goal: L{landmarkIDs[current_goal]}")
        r.r.stop()

if __name__ == '__main__':
    main()
