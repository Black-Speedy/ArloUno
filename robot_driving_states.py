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
    SETUP = 5
    FINDPOS = 6

class RobotController():
    def __init__(self, path, x = 0, y = 0, theta = np.pi / 2, FollowRRT = True):
        self.r = rb.Robot()
        self.ds = DriveState.SETUP
        self.stopTimer = 0
        self.stopTurnTimer = 0
        self.path = np.flip(np.array(path), 0)
        self.theta = theta
        self.currentPoint = 0
        self.waitTime = 0.0
        self.x = x
        self.y = y
        self.FollowRRT = FollowRRT
    
    def straight64(self, cm):
        self.r.go_diff(65, 70, 1, 1)
        self.x = self.x + np.cos(self.theta) * cm
        self.y = self.y + np.sin(self.theta) * cm
        self.stopTimer = time.perf_counter() + (cm * 2.24 * 1.25) / 100

    def turnDegree(self, degrees, direction):
        if direction == "left":
            self.r.go_diff(30, 30, 0, 1)
            self.theta += np.deg2rad(degrees)
            self.stopTurnTimer = time.perf_counter() + (degrees / 90) * 1.95 * 2 #1.95
        else:
            self.r.go_diff(30, 30, 1, 0)
            self.theta -= np.deg2rad(degrees)
            self.stopTurnTimer = time.perf_counter() + (degrees / 90) * 1.75 * 2 #1.95

    def update(self):
        if (self.ds == DriveState.SETUP):
            # Drive half robots length forward, to translate the robots position into the cameras.
            self.ds = DriveState.STRAIGHT
            self.straight64(22)

        if (self.ds == DriveState.STOP):
            if (self.stopTimer < time.perf_counter()):
                if self.FollowRRT:
                    self.ds = DriveState.SEARCH

        elif (self.ds == DriveState.TURN):
            if (self.stopTurnTimer < time.perf_counter()):
                if self.FollowRRT:
                    self.ds = DriveState.STOP
                    self.stopTimer = time.perf_counter() + 0.8
                    self.r.stop()
                else:
                    self.ds = DriveState.STOP
                    self.stopTimer = time.perf_counter() + 0.8
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
            
            ydiff = self.path[self.currentPoint + 1][1] - self.path[self.currentPoint][1]
            xdiff = self.path[self.currentPoint + 1][0] - self.path[self.currentPoint][0]

            # find angle to next point
            theta = np.arctan2(ydiff, xdiff) - (self.theta)

            thetaDegrees = np.rad2deg(theta)
            print("")
            print(f"current target point {self.currentPoint}")
            print("theta to turn: " + str(thetaDegrees))
            print(f"robots theta: {np.rad2deg(self.theta)}")

            if ((0.001 > thetaDegrees > -0.001) or (thetaDegrees > 359.999) or (thetaDegrees < -359.999)):
                # we need to drive straight
                self.ds = DriveState.STRAIGHT
                self.straight64(np.linalg.norm(self.path[self.currentPoint + 1] - self.path[self.currentPoint])* 100)
                print(f"drive straight for {np.linalg.norm(self.path[self.currentPoint + 1] - self.path[self.currentPoint])* 100:.2} cm")
                self.currentPoint += 1
            else:
                # we need to turn
                self.ds = DriveState.TURN
                if (thetaDegrees > 0):
                    if thetaDegrees > 180:
                        print(f"turning right {thetaDegrees - 180}")
                        self.turnDegree(thetaDegrees - 180, "right")
                    else:
                        print(f"turning left {thetaDegrees}")
                        self.turnDegree(thetaDegrees, "left")
                else:
                    if -180 < thetaDegrees < 0:
                        print(f"turning right {np.abs(thetaDegrees)}")
                        self.turnDegree(np.abs(thetaDegrees), "right")
                    elif thetaDegrees <= -180:
                        print(f"turning left {np.abs(thetaDegrees) - 180}")
                        self.turnDegree(np.abs(thetaDegrees) - 180, "left")


        elif (self.ds == DriveState.EXIT):
            print("EXIT!")
            exit()
        
        """ if (self.theta > 360 or self.theta < 0):
            print(f"regulating theta from {self.theta} to {self.theta % 360}")
            self.theta = self.theta % 360 """