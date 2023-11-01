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


landmarkIDs = [5, 6, 2, 51]
landmarks = {
    5: (0.0, 0.0),  # Coordinates for landmark 1
    6: (0.0, 300.0),  # Cordinates for landmark 2
    2: (200.0, 0.0),
    51: (400.0, 300.0)
}
landmark_dists = {
    5: -1,
    6: -1,
    2: -1,
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
    distance_to_A = landmark_dists[5] + 22.5  # Distance to Landmark A
    distance_to_B = landmark_dists[6] + 22.5  # Distance to Landmark B
    dAB = 200.0  # Distance between Landmark A and B

    eps = 0.01
    # Calculate robot's position
    cos_theta = (distance_to_A**2 - distance_to_B **2 + dAB**2) / (2 * dAB * distance_to_A + eps)
    print(f"before cos theta: {cos_theta}")
    cos_theta = min(cos_theta, 0.999)
    print(f"after cos theta: {cos_theta}")

    theta = math.acos(cos_theta)

    y = distance_to_A * math.cos(theta)
    x = distance_to_A * math.sin(theta)


    if landmarks_found[1] == 6: # We found L1 then L2
        if theta_turned < 180:
            x = -x

    elif landmarks_found[1] == 50: # We found L1 then L3
        if theta_turned > 180:
            y = -y

    print(f"theta turned: {theta_turned}")
    print(f"distance to a: {distance_to_A}, distance to b: {distance_to_B}, theta: {theta}\n robot pose x: {x}, y: {y}")

    exit()





    pos = selflocalize.Localize(cam)

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   #
    path_res = 0.05
    map = grid_occ.GridOccupancyMap(low=(-6, -6), high=(6, 6), res=path_res, cam=cam)
    map.populate()


    # Get start position and robot theta

    print(f"Robot pose: x: {pos.getX()} y: {pos.getY()} theta: {np.rad2deg(pos.getTheta())}")

    rrt = RRT(
        start=[pos.getX() / 100, pos.getY() / 100],
        goal=[0.5, 0],
        robot_model=robot,
        map=map,
        expand_dis=1.0,
        path_resolution=path_res,
    )

    print("3")
    show_animation = False
    metadata = dict(title="RRT Test")
    writer = FFMpegWriter(fps=15, metadata=metadata)
    fig = plt.figure()
    path = rrt.planning(animation=show_animation, writer=writer)

    with writer.saving(fig, "rrt_test.mp4", 100):

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                # write the node number

                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()
                writer.grab_frame()

    rrt.draw_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    pathPrint = np.flip(np.array(path))  # might need np.array here
    for i in range(0, len(pathPrint)):
        plt.text(pathPrint[i][1], pathPrint[i][0], str(
            i), color="blue", fontsize=10)

    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.savefig("map_with_path.png")

    # Use github to push the image


    maxTime = 60 + time.perf_counter()

    ctime = time.perf_counter()
    while (maxTime > time.perf_counter()):
        # only update every 0.1 seconds
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()


if __name__ == '__main__':
    main()
