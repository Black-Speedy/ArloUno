from robot import *
r = RobotController()

maxTime = 60 + time.perf_counter()

ctime = time.perf_counter()
while(maxTime > time.perf_counter()):
    # only update every 0.1 seconds
    if ctime + 0.1 < time.perf_counter():
        print(r.ds)
        r.update()
        ctime = time.perf_counter()