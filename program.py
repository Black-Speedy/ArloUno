import robot
from robot import *
r = robot.RobotController()

maxTime = 60 + time.perf_counter()
while(maxTime > time.perf_counter()):
    r.update()