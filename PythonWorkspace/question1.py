import vrep #import library for VREP API
import time
import math
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting
import signal

from idash import IDash
from robot_helpers import vomega2bytecodes

class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self,signum, frame):
        self.kill_now = True

class FinalProjectProgram():

    def __init__(self, color, number):
        # parameter init
        self.bot_name = '%s%d' % (color, number)

        # run startup methods
        self.initialize_vrep_client()
        self.initilize_vrep_api()
        self.killer = GracefulKiller()
        self.idash = IDash(framerate=0.05)

    def initialize_vrep_client(self):
        #Initialisation for Python to connect to VREP
        print 'Python program started'
        count = 0
        num_tries = 10
        while count < num_tries:
            vrep.simxFinish(-1) # just in case, close all opened connections
            self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #Timeout=5000ms, Threadcycle=5ms
            if self.clientID!=-1:
                print 'Connected to V-REP'
                break
            else:
                "Trying again in a few moments..."
                time.sleep(3)
                count += 1
        if count >= num_tries:
            print 'Failed connecting to V-REP'
            vrep.simxFinish(self.clientID)

    def initilize_vrep_api(self):
        # initialize bot handles and variables
        _, self.leftMotor=vrep.simxGetObjectHandle(
            self.clientID, '%s_leftJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.rightMotor=vrep.simxGetObjectHandle(
            self.clientID, '%s_rightJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.bot=vrep.simxGetObjectHandle(
            self.clientID, self.bot_name, vrep.simx_opmode_oneshot_wait)
        # proxSens = prox_sens_initialize(self.clientID)
        # initialize odom of bot
        # _, self.xyz = vrep.simxGetObjectPosition(
        #     self.clientID, self.bot, -1, vrep.simx_opmode_streaming)
        # _, self.eulerAngles = vrep.simxGetObjectOrientation(
        #     self.clientID, self.bot, -1, vrep.simx_opmode_streaming)

        # # initialize overhead cam
        # _, self.overheadCam=vrep.simxGetObjectHandle(
        #     self.clientID, 'Global_Vision', vrep.simx_opmode_oneshot_wait)
        # _, self.resolution, self.image = vrep.simxGetVisionSensorImage(
        #     self.clientID,self.overheadCam,0,vrep.simx_opmode_oneshot_wait)

        # # initialize goal handle + odom
        # _, self.goalHandle=vrep.simxGetObjectHandle(
        #     self.clientID, 'Goal_Position', vrep.simx_opmode_oneshot_wait)
        # _, self.goalPose = vrep.simxGetObjectPosition(
        #     self.clientID, self.goalHandle, -1, vrep.simx_opmode_streaming)

    def robot_code(self):
        """ OUR ROBOT CODE GOES HERE """
        while True:
            self.unittest_move_forward()

    def unittest_move_forward(self):
        forward_vel = 1
        omega = 0
        ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g=1)
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity


    def clean_exit(self):
        _ = vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
        vrep.simxFinish(self.clientID)
        print 'Program ended'

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
            self.robot_code()

        self.clean_exit()

if __name__ == '__main__':
    obj = FinalProjectProgram('Blue', 1)
    obj.run()