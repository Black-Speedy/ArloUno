import sys
import time
import enum
from time import sleep
import serial
import numpy as np
from picamera2 import Picamera2
from robot import *

class DriveState(enum):
    STOP = 0
    STRAIGHT = 1
    TURN = 2


r = Robot()
xPos = 0
yPos = 0
theta = 0  # Angle in radians
rSize = 400  # Robot size in mm
targetX = 3000
targetY = 3000
ds = DriveState.STOP



sr = [] # Sensor readings, as x,y coordinates

def read_sensors():
    reading = r.read_front_ping_sensor()
    if reading > 0 and reading < 3500:
        sr.append((xPos + reading * np.cos(theta), yPos + reading * np.sin(theta)))
    reading = r.read_left_ping_sensor()
    if reading > 0 and reading < 3500:
        sr.append((xPos + reading * np.cos(theta - np.pi/4), yPos + reading * np.sin(theta - np.pi/4)))
    reading = r.read_right_ping_sensor()
    if reading > 0 and reading < 3500:
        sr.append((xPos + reading * np.cos(theta + np.pi/4), yPos + reading * np.sin(theta + np.pi/4)))
    reading = r.read_back_ping_sensor()
    if reading > 0 and reading < 3500:
        sr.append((xPos + reading * np.cos(theta + np.pi), yPos + reading * np.sin(theta + np.pi)))

def turn(r, degrees, direction):
    radians = degrees * np.pi / 180
    global ds, stopTurnTimer
    if radians < 0:
        radians = radians + 2 * np.pi
    if direction == "left":
        r.go_diff(30, 30, 0, 1)
        theta += radians
    else:
        r.go_diff(30, 30, 1, 0)
        theta -= radians
    ds = DriveState.TURN
    stopTurnTimer = time.perf_counter() + (degrees / 90) * 1.95 

def drive_forward(r):
    r.go_diff(65, 70, 1, 1)

stopTurnTimer = 0
turn_time = input("Input turn time: ")
dist = int(input("Input distance: "))
distSide = int(input("Input side distance: "))
isDriving = True
turning = False
avoiding = False
lastSensorReadTime = 0
start = time.perf_counter()
while (isDriving): 
    if (ds == DriveState.STRAIGHT):
        if (r.read_front_ping_sensor() < dist):
            r.stop()
            avoiding = True
            ds = DriveState.TURN
            if (r.read_left_ping_sensor() < distSide):
                turn(r, 90, "right")
            elif (r.read_right_ping_sensor() < distSide):
                turn(r, 90, "left")
            elif (r.read_front_ping_sensor() < dist):
                # choose a random direction from left and right
                random_direction = np.random.choice(
                    ["left", "right"], p=[0.5, 0.5])
                turn(r, 90, random_direction)
                print("turning random")
    
    if (ds == DriveState.TURN):
        if (time.perf_counter() > stopTurnTimer):
            r.stop()
            if avoiding == False:
                ds = DriveState.STOP
            else:
                drive_forward(r)
                ds = DriveState.STRAIGHT

    # Stop after 60 seconds
    if (time.perf_counter() - start > 60):
        print(r.stop())
        isDriving = False

    # Read sensors every second
    if (time.perf_counter() - lastSensorReadTime > 0.5):
        lastSensorReadTime = time.perf_counter()
        read_sensors()

    # rotate towards target
    if (ds == DriveState.STOP):
        # Calculate angle to target
        angle = np.arctan2(targetY - yPos, targetX - xPos)
        # Calculate shortest angle to target
        angleDiff = angle - theta
        if (angleDiff > np.pi):
            angleDiff -= 2 * np.pi
        elif (angleDiff < -np.pi):
            angleDiff += 2 * np.pi
        # Rotate towards target
        if (np.abs(angleDiff) > 0.1):
            ds = DriveState.TURN
            if (angleDiff > 0):
                turn(r, angleDiff, "left")
            else:
                turn(r, angleDiff, "right")
        else:
            ds = DriveState.STRAIGHT
            r.drive_forward()





    """ if (turning):
        if time.perf_counter() - turnTimer > float(turn_time):
            turning = False
            straight64(r) """

    

    """ elif (r.read_left_ping_sensor() < distSide):
        turning = True
        turnTimer = time.perf_counter()
        turn(r, "right")
        print("turning right")
    elif (r.read_right_ping_sensor() < distSide):
        turning = True
        turnTimer = time.perf_counter()
        turn(r, "left")
        print("turning left")
    elif (r.read_front_ping_sensor() < dist):
        turning = True
        turnTimer = time.perf_counter()
        # choose a random direction from left and right
        random_direction = np.random.choice(
            ["left", "right"], p=[0.5, 0.5])
        turn(r, random_direction)
        print("turning random") """

