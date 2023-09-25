# Arlo Robot Controller
import sys
import time
from time import sleep
import serial
import numpy as np
import picamera2
from picamera2 import Picamera2
from enum import Enum
import landMark as e1
from landMark import *
import cv2 # Import the OpenCV library
from pprint import *

# 
class DriveState(Enum):
    STOP = 0
    STRAIGHT = 1
    TURN = 2
    SEARCH = 3
    EXIT = 4


class Robot(object):
    """Defines the Arlo robot API
    
       DISCLAIMER: This code does not contain error checking - it is the responsibility
       of the caller to ensure proper parameters and not to send commands to the 
       Arduino too frequently (give it time to process the command by adding a short sleep wait
       statement). Failure to do some may lead to strange robot behaviour.
       
       In case you experience trouble - consider using only commands that do not use the wheel 
       encoders.
    """ 
    def __init__(self, port = '/dev/ttyACM0'):
        """The constructor port parameter can be changed from default value if you want
           to control the robot directly from your labtop (instead of from the on-board raspberry 
           pi). The value of port should point to the USB port on which the robot Arduino is connected."""
        self.port = port
        
        #self.serialRead = serial.Serial(self.port,9600, timeout=1) # 1 sec. timeout, wait until data is received or until timeout
        self.serialRead = serial.Serial(self.port,9600, timeout=None) # No timeout, wait forever or until data is received

        # Wait if serial port is not open yet
        while not self.serialRead.isOpen():
            sleep(1)

        print("Waiting for serial port connection ...")
        sleep(2)

        print("Running ...")
        
    def __del__(self):
        print("Shutting down the robot ...")
        
        sleep(0.05)
        print(self.stop())
        sleep(0.1)
                
        cmd='k\n'
        print((self.send_command(cmd)))
        self.serialRead.close()
        
        
    def send_command(self, cmd, sleep_ms=0.0):
        """Sends a command to the Arduino robot controller"""
        self.serialRead.write(cmd.encode('ascii'))
        sleep(sleep_ms)
        str_val=self.serialRead.readline()
        return str_val


    def _power_checker(self, power):
        """Checks if a power value is in the set {0, [30;127]}.
           This is an internal utility function."""
        return  (power == 0) or (power >=30 and power <=127) 

        
    def go_diff(self, powerLeft, powerRight, dirLeft, dirRight):
        """Start left motor with motor power powerLeft (in {0, [30;127]} and the numbers must be integer) and direction dirLeft (0=reverse, 1=forward)
           and right motor with motor power powerRight (in {0, [30;127]} and the numbers must be integer) and direction dirRight (0=reverse, 1=forward).
        
           The Arlo robot may blow a fuse if you run the motors at less than 40 in motor power, therefore choose either 
           power = 0 or 30 < power <= 127.
           
           This does NOT use wheel encoders."""
        
        if (not self._power_checker(powerLeft)) or (not self._power_checker(powerRight)):
            print("WARNING: Read the docstring of Robot.go_diff()!")
            return ""
        else:
            cmd = 'd' + str(int(powerLeft)) + ',' + str(int(powerRight)) + ',' + str(int(dirLeft)) + ',' + str(int(dirRight)) + '\n'
            return self.send_command(cmd)


    def stop(self):
        """Send a stop command to stop motors. Sets the motor power on both wheels to zero.
        
           This does NOT use wheel encoders."""
        cmd='s\n'
        return self.send_command(cmd)


    
    def read_sensor(self, sensorid):
        """Send a read sensor command with sensorid and return sensor value. 
           Will return -1, if error occurs."""
        cmd=str(sensorid) + '\n'
        str_val=self.send_command(cmd)
        if len(str_val) > 0:
            return int(str_val)
        else:
            return -1
            
    def read_front_ping_sensor(self):
        """Read the front sonar ping sensor and return the measured range in milimeters [mm]"""
        return self.read_sensor(0)
        
    def read_back_ping_sensor(self):
        """Read the back sonar ping sensor and return the measured range in milimeters [mm]"""
        return self.read_sensor(1)
        
    def read_left_ping_sensor(self):
        """Read the left sonar ping sensor and return the measured range in milimeters [mm]"""
        return self.read_sensor(2)
        
    def read_right_ping_sensor(self):
        """Read the right sonar ping sensor and return the measured range in milimeters [mm]"""
        return self.read_sensor(3)

    
    def read_left_wheel_encoder(self):
        """Reads the left wheel encoder counts since last reset_encoder_counts command.
           The encoder has 144 counts for one complete wheel revolution."""
        cmd='e0\n'
        return self.send_command(cmd, 0.045)

    def read_right_wheel_encoder(self):
        """Reads the right wheel encoder counts since last clear reset_encoder_counts command.
           The encoder has 144 counts for one complete wheel revolution."""
        cmd='e1\n'
        return self.send_command(cmd, 0.045)

    def reset_encoder_counts(self):
        """Reset the wheel encoder counts."""
        cmd='c\n'
        return self.send_command(cmd)
    
    
    ### OBSOLETE STUFF
        
    def go(self):
        """OBSOLETE: Send a go command for continuous forward driving using the wheel encoders"""
        cmd='g\n'
        return self.send_command(cmd)
        
    def backward(self):
        """OBSOLETE: Send a backward command for continuous reverse driving using the wheel encoders"""
        cmd='v\n'
        return self.send_command(cmd)
        

    def left(self):
        """OBSOLETE: Send a rotate left command for continuous rotating left using the wheel encoders"""
        cmd='n\n'
        return self.send_command(cmd)

    def right(self):
        """OBSOLETE: Send a rotate right command for continuous rotating right using the wheel encoders"""
        cmd='m\n'
        return self.send_command(cmd)
        
    def step_forward(self):
        """OBSOLETE: Send a step forward command for driving forward using the wheel encoders for a 
           predefined amount of time"""
        cmd='f\n'
        return self.send_command(cmd)

    def step_backward(self):
        """OBSOLETE: Send a step backward command for driving backward using the wheel encoders for a 
           predefined amount of time"""
        cmd='b\n'
        return self.send_command(cmd)

    def step_rotate_left(self):
        """OBSOLETE: Send a step rotate left command for rotating left using the wheel encoders for a 
           predefined amount of time"""
        cmd='l\n'
        return self.send_command(cmd)
        
    def step_rotate_right(self):
        """OBSOLETE: Send a step rotate right command for rotating right using the wheel encoders for 
           a predefined amount of time"""
        cmd='r\n'
        return self.send_command(cmd)
            
        
    def set_speed(self, speed):
        """OBSOLETE: Speed must be a value in the range [0; 255]. This speed is used in commands based on 
           using the wheel encoders."""
        cmd='z' + str(speed) + '\n'
        return self.send_command(cmd)
        
    def set_turnspeed(self, speed):
        """OBSOLETE: Turnspeed must be a value in the range [0; 255]. This speed is used in commands based on 
           using the wheel encoders."""
        cmd='x' + str(speed) + '\n'
        return self.send_command(cmd)

    def set_step_time(self, steptime):
        """OBSOLETE: steptime is the amount of miliseconds used in the step_forward and step_backwards 
           commands."""
        cmd='t' + str(steptime) + '\n'
        return self.send_command(cmd)
        
    def set_turn_time(self, turntime):
        """OBSOLETE: turntime is the amount of miliseconds used in the step_rotate_left and 
        step_rotate_right commands."""
        cmd='y' + str(turntime) + '\n'
        return self.send_command(cmd)
        

    def turn(self, direction):
        if (direction == "left"):
            self.go_diff(30, 30, 0, 1)
        else:
            self.go_diff(30, 30, 1, 0)


    def sv(int):
        if int > 127:
            return 127
        elif int < 30:
            return 30
        else:
            return int


    def smoothTurn(r, direction):
        if (direction == "left"):
            r.go_diff(38, 84, 1, 1)
            sleep(8.75)
        else:
            r.go_diff(81, 40, 1, 1)
            sleep(8.3)



class RobotController():
    def __init__(self):
        self.r = Robot()
        self.ds = DriveState.SEARCH
        self.stopTimer = 0
        self.stopTurnTimer = 0
        self.box = (0, 0, 0)
        self.boxFound = False
    
    def straight64(self, cm):
        self.r.go_diff(65, 70, 1, 1)
        self.stopTimer = time.perf_counter() + cm * 2.24 / 100

    def turnDegree(self, degrees, direction):
        print("turning")
        theta = 0
        radians = degrees * np.pi / 180
        if radians < 0:
            radians = radians + 2 * np.pi
        if direction == "left":
            self.r.go_diff(30, 30, 0, 1)
            theta += radians
        else:
            self.r.go_diff(30, 30, 1, 0)
            theta -= radians
        self.stopTurnTimer = time.perf_counter() + (degrees / 90) * 1.95

    def update(self):
        if (self.ds == DriveState.STOP):
            if (self.stopTimer < time.perf_counter()):
                if self.boxFound:
                    self.ds = DriveState.STRAIGHT
                    self.straight64(int(self.box[0]*100) - 40)
                else:
                    self.ds = DriveState.SEARCH
        elif (self.ds == DriveState.TURN):
            if (self.stopTurnTimer < time.perf_counter()):
                self.ds = DriveState.STOP
                self.stopTimer = time.perf_counter() + 0.3
                self.r.stop()
        elif (self.ds == DriveState.STRAIGHT):
            if(self.stopTimer < time.perf_counter()):
                self.r.stop()
                self.ds = DriveState.EXIT
        elif (self.ds == DriveState.SEARCH):
            self.locateBox()
        elif (self.ds == DriveState.EXIT):
            print("EXIT!")
            exit()

    def locateBox(self):
        result = e1.lookBox(8)
        result = e1.lookBox(8)
        result = e1.lookBox(8)
        result = e1.lookBox(8)
        result = e1.lookBox(8)
        result = e1.lookBox(8)
        # split result into (x, y, z) and ids
        dist = result[0]
        xDegrees = result[1]
        boxDegrees = result[2]
        ids = result[3]

        #image = cam.capture_array("main")
        """ cnt += 1
        cv2.imwrite("test" + str(cnt) + ".jpg", image) """
        if dist != 0.0:
            # if ids contains # 8:
            if 8 in ids:
                print("IM HERE")
                print(result)
                self.box = (dist, xDegrees, boxDegrees)
                self.r.stop()
                self.ds = DriveState.TURN
                self.boxFound = True
                if xDegrees > 0:
                    self.turnDegree(xDegrees, "right")
                else:
                    self.turnDegree(xDegrees, "left")
                return result
            else:
                print("found wrong box?")
            
        
        self.ds = DriveState.TURN
        self.turnDegree(8, "left")
   

                

            

        
