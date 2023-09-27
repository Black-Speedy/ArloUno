from enum import Enum
import robot as rb
import time
import numpy as np

class DriveState(Enum):
    STOP = 0
    STRAIGHT = 1
    TURN = 2
    SEARCH = 3
    EXIT = 4

class RobotController():
    def __init__(self):
        self.r = rb.Robot()
        self.ds = DriveState.SEARCH
        self.stopTimer = 0
        self.stopTurnTimer = 0
        self.path = []
        self.theta = 0
        self.currentPoint = 0
    
    def straight64(self, cm):
        self.r.go_diff(65, 70, 1, 1)
        self.stopTimer = time.perf_counter() + cm * 2.24 / 100

    def turnDegree(self, degrees, direction, convert=False):
        print("turning")
        if convert:
            radians = degrees * np.pi / 180
        else:
            radians = degrees
        if radians < 0:
            radians = radians + 2 * np.pi
        if direction == "left":
            self.r.go_diff(30, 30, 0, 1)
            self.theta += radians
        else:
            self.r.go_diff(30, 30, 1, 0)
            self.theta -= radians
        self.stopTurnTimer = time.perf_counter() + (degrees / 90) * 1.95

    def update(self):
        if (self.ds == DriveState.STOP):
            if (self.stopTimer < time.perf_counter()):
                self.ds = DriveState.SEARCH

        elif (self.ds == DriveState.TURN):
            if (self.stopTurnTimer < time.perf_counter()):
                self.ds = DriveState.STOP
                self.stopTimer = time.perf_counter() + 0.3
                self.r.stop()

        elif (self.ds == DriveState.STRAIGHT):
            if(self.stopTimer < time.perf_counter()):
                self.r.stop()
                self.ds = DriveState.STOP
                self.stopTimer = time.perf_counter() + 0.3

        elif (self.ds == DriveState.SEARCH):
            if self.currentPoint == len(self.path) - 1:
                self.ds = DriveState.EXIT
                return
            
            # find angle to next point
            theta = np.arctan2(self.path[self.currentPoint + 1][1] - self.path[self.currentPoint][1],
                                 self.path[self.currentPoint][0] - self.path[self.currentPoint][0]) - self.theta

            if (theta != 0):
                # we need to turn
                self.ds = DriveState.TURN
                if (theta > 0):
                    self.turnDegree(np.degrees(theta), "left")
                else:
                    self.turnDegree(np.degrees(theta), "right")
            else:
                # we need to drive straight
                self.ds = DriveState.STRAIGHT
                self.straight64(np.linalg.norm(self.path[self.currentPoint + 1] - self.path[self.currentPoint]))
                self.currentPoint += 1

        elif (self.ds == DriveState.EXIT):
            print("EXIT!")
            exit()