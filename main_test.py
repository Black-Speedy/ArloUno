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

if __name__ == '__main__':
    while True:
        main()
