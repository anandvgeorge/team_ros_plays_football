"""
Path follower test
Romain Chiappinelli
04.03.16
"""
import abc
import vrep #import library for VREP API
import time
import math
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting
import signal

from idash import IDash
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos
from plot_helpers import plotVector

z = 0.027536552399396896 # z-Position of robot

class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self,signum, frame):
        self.kill_now = True

class BaseRobotRunner(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, color, number):
        # parameter init
        self.bot_name = '%s%d' % (color, number)
        self.bot_nameStriker = 'Red1'

        # run startup methods
        self.initializeVrepClient()
        self.initializeVrepApi()
        self.killer = GracefulKiller()
        self.idash = IDash(framerate=0.05)

    def initializeVrepClient(self):
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

    def initializeVrepApi(self):
        # initialize bot handles and variables
        _, self.leftMotor=vrep.simxGetObjectHandle(
            self.clientID, '%s_leftJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.rightMotor=vrep.simxGetObjectHandle(
            self.clientID, '%s_rightJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.bot=vrep.simxGetObjectHandle(
            self.clientID, self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.ball=vrep.simxGetObjectHandle(
            self.clientID, 'Ball', vrep.simx_opmode_oneshot_wait)

        # proxSens = prox_sens_initialize(self.clientID)
        # initialize odom of bot
        _, self.xyz = vrep.simxGetObjectPosition(
            self.clientID, self.bot, -1, vrep.simx_opmode_streaming)
        _, self.eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, self.bot, -1, vrep.simx_opmode_streaming)

        # initialize bot handles and variables for Red1
        _, self.leftMotorStriker=vrep.simxGetObjectHandle(
            self.clientID, '%s_leftJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.rightMotorStriker=vrep.simxGetObjectHandle(
            self.clientID, '%s_rightJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.botStriker = vrep.simxGetObjectHandle(
            self.clientID, self.bot_nameStriker, vrep.simx_opmode_oneshot_wait)
        _, xyzStriker = vrep.simxGetObjectPosition(
            self.clientID, self.botStriker, -1, vrep.simx_opmode_streaming)
        _, eulerAnglesStriker = vrep.simxGetObjectOrientation(
            self.clientID, self.botStriker, -1, vrep.simx_opmode_streaming)
        # get ball position
        _, ballxyz = vrep.simxGetObjectPosition(
            self.clientID, self.ball, -1, vrep.simx_opmode_streaming)

    @abc.abstractmethod
    def robotCode(self):
        """ OUR ROBOT CODE GOES HERE """
        return

    def getRobotConf(self, robot_handle):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]

        return (x, y, theta)

    def getBallPose(self):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, self.ball, -1, vrep.simx_opmode_streaming)
        x, y, z = xyz
        return (x, y)

    def setMotorVelocities(self, forward_vel, omega):
        ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g=1)
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity

    def unittestMoveForward(self):
        self.setMotorVelocities(forward_vel=1, omega=0)

    def unittestTurnSideways(self, not_first_time):
        x, y, theta = self.getRobotPose()
        if not_first_time:
            goal_theta = self.first_theta + np.pi / 2
            print goal_theta
            error_theta = ThetaRange.angleDiff(theta, goal_theta)

            # control
            omega = 10 * error_theta
            print omega
            self.setMotorVelocities(0, omega)

            # visualization
            def plotCurrentDesiredHeadings():
                plotVector(ThetaRange.pol2cart(1, theta), 'k')
                plotVector(ThetaRange.pol2cart(1, goal_theta), 'r')
            self.idash.add(plotCurrentDesiredHeadings)
            self.idash.plotframe()
        else:
            self.first_theta = theta

    def clean_exit(self):
        _ = vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
        vrep.simxFinish(self.clientID)
        print 'Program ended'

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
            self.robotCode()

        self.clean_exit()


if __name__ == '__main__':
    class MyRobotRunner(BaseRobotRunner):

        def __init__(self, *args, **kwargs):
            super(MyRobotRunner, self).__init__(*args, **kwargs)

        def robotCode(self):
            while 1:
                robotConf = self.getRobotConf(self.bot)
                print('robotConf.x=%f' % robotConf[0])
                print('robotConf.y=%f' % robotConf[1])
                print('robotConf.theta=%f' % robotConf[2])
                ballPos = self.getBallPose() # (x, y)
                print('ballPos.x=%f' % ballPos[0])
                print('ballPos.y=%f' % ballPos[1])
                vRobot = v2Pos(robotConf, ballPos)
                print('vRobot.x=%f' % vRobot[0])
                print('vRobot.y=%f' % vRobot[1])
                self.setMotorVelocities(vRobot[0], vRobot[1])
            time.sleep(3)

    obj = MyRobotRunner('Blue', 1)
    obj.run()
