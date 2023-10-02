from robot_driving_states import RobotController
from rrt import *
import time

def main():

    path_res = 0.05
    map = grid_occ.GridOccupancyMap(low=(-1, 0), high=(1, 2), res=path_res)
    map.populate()

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   #

    rrt = RRT(
        start=[0, 0],
        goal=[0, 1.9],
        robot_model=robot,
        map=map,
        expand_dis=0.6,
        path_resolution=path_res,
    )

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
    for i in range(0, len(path)):
        plt.text(path[i][0], path[i][1], str(
            i), color="blue", fontsize=10)
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.savefig("map_with_path.png")

    
    r = RobotController(path)

    maxTime = 60 + time.perf_counter()


    ctime = time.perf_counter()
    while (maxTime > time.perf_counter()):
        # only update every 0.1 seconds
        if ctime + 0.1 < time.perf_counter():
            r.update()
            ctime = time.perf_counter()


if __name__ == '__main__':
    main()
