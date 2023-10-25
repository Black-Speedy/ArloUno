from robot_driving_states import RobotController
from rrt import *
import sys
import os
sys.path.append(("Self-localization"))
import selflocalize
import camera
import time


def main():

    print("-1")
    cam = camera.Camera(0, 'arlo', useCaptureThread=True)
    print("-0.5")

    pos = selflocalize.Localize(cam)

    print("0")
    path_res = 0.05
    map = grid_occ.GridOccupancyMap(low=(-6, -6), high=(6, 6), res=path_res, cam=cam)
    map.populate()

    print("1")
    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   #
    print("2")



    # Get start position and robot theta

    rrt = RRT(
        start=[pos.getX() * 100, pos.getY() * 100],
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

    r = RobotController(path, x=pos.getX() * 100, y=pos.getY() * 100, theta=pos.getTheta())

    maxTime = 60 + time.perf_counter()

    ctime = time.perf_counter()
    while (maxTime > time.perf_counter()):
        # only update every 0.1 seconds
        if ctime + 0.001 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()


if __name__ == '__main__':
    main()
