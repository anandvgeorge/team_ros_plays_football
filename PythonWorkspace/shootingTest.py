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

class FinalProjectProgram():

    def __init__(self, color, number):
        # parameter init
        self.bot_name = '%s%d' % (color, number)
        self.bot_player2 = 'Blue2'
        self.bot_nameStriker = 'Red1'
        self.bot_nameGoalie = 'Red3'
        self.bot_nameBall = 'Ball'

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
        # proxSens = prox_sens_initialize(self.clientID)
        # initialize odom of bot
        _, self.xyz = vrep.simxGetObjectPosition(
            self.clientID, self.bot, -1, vrep.simx_opmode_streaming)
        _, self.eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, self.bot, -1, vrep.simx_opmode_streaming)
            
        _, self.player2=vrep.simxGetObjectHandle(
            self.clientID, self.bot_player2, vrep.simx_opmode_oneshot_wait)
        # proxSens = prox_sens_initialize(self.clientID)
        # initialize odom of bot
        _, self.xyzPlayer2 = vrep.simxGetObjectPosition(
            self.clientID, self.player2, -1, vrep.simx_opmode_streaming)
        _, self.eulerAnglesPlayer2 = vrep.simxGetObjectOrientation(
            self.clientID, self.player2, -1, vrep.simx_opmode_streaming)

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
            
        _, self.leftMotorGoalie =vrep.simxGetObjectHandle(
            self.clientID, '%s_leftJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.rightMotorGoalie =vrep.simxGetObjectHandle(
            self.clientID, '%s_rightJoint' % self.bot_name, vrep.simx_opmode_oneshot_wait)
        _, self.botGoalie = vrep.simxGetObjectHandle(
            self.clientID, self.bot_nameGoalie, vrep.simx_opmode_oneshot_wait)
        _, xyzGoalie = vrep.simxGetObjectPosition(
            self.clientID, self.botGoalie, -1, vrep.simx_opmode_streaming)
        _, eulerAnglesGoalie = vrep.simxGetObjectOrientation(
            self.clientID, self.botGoalie, -1, vrep.simx_opmode_streaming)
            
        _, self.ball = vrep.simxGetObjectHandle(
            self.clientID, self.bot_nameBall, vrep.simx_opmode_oneshot_wait)
        _, xyzBall = vrep.simxGetObjectPosition(
            self.clientID, self.ball, -1, vrep.simx_opmode_streaming)
        _, eulerAnglesBall = vrep.simxGetObjectOrientation(
            self.clientID, self.ball, -1, vrep.simx_opmode_streaming)

    def robotCode(self):
        """ OUR ROBOT CODE GOES HERE """
        # Shooting Test
        
        vrep.simxSetObjectPosition(
            self.clientID, self.ball,-1,[0, -0.35], vrep.simx_opmode_streaming)  
            
        target = self.aim()
        
        vrep.simxSetObjectPosition(
        self.clientID, self.player2, -1, target, vrep.simx_opmode_streaming)
        
        position = []
        position = self.kickingPose(target)
        kickingPosition = [position[0], position[1], z]
        vrep.simxSetObjectPosition(
            self.clientID, self.bot, -1, kickingPosition, vrep.simx_opmode_streaming)
        vrep.simxSetObjectOrientation(
            self.clientID, self.bot, -1, [self.eulerAngles[0], self.eulerAngles[1], position[2]], vrep.simx_opmode_streaming)
        
        time.sleep(2)
        
        # Testing kicking Positions
#        vrep.simxSetObjectPosition(
#            self.clientID, self.ball,-1,[0, 0], vrep.simx_opmode_streaming)
#        
#        maximum = 36
#        for i in range (0,maximum):
#            print i
#            radius = 0.3
#            angle = 2*math.pi/maximum*i
#            target = [radius*math.sin(angle), radius*math.cos(angle)]
#            
#            vrep.simxSetObjectPosition(
#            self.clientID, self.player2, -1, target, vrep.simx_opmode_streaming)
#            
#            position = []
#            position = self.kickingPose(target)
#            kickingPosition = [position[0], position[1], z]
#            vrep.simxSetObjectPosition(
#                self.clientID, self.bot, -1, kickingPosition, vrep.simx_opmode_streaming)
#            vrep.simxSetObjectOrientation(
#                self.clientID, self.bot, -1, [self.eulerAngles[0], self.eulerAngles[1], position[2]], vrep.simx_opmode_streaming)
#            
#            time.sleep(1)
        
        
    def getRobotConf(self, robot_handle):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]

        return (x, y, theta)

    def getRobotPose(self, robot_handle):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]

        return (x, y, theta)

    def setMotorVelocities(self, forward_vel, omega):
        ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g=1)
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity
            
    def getBallPose(self):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, self.ball, -1, vrep.simx_opmode_streaming)
        x, y, z = xyz
        return (x, y)
        
    def aim(self, goaliePosition):
        # assuming: robot is already in perfect position
        # assuming: ball has velocity of zero
        ballRadius = 0.05
        leftGoalPost = [0.2, -0.75] # position of left goal post
        rightGoalPost = [-0.2, 0.75]# position of right goal post       
        tolerance = 0.01
        
#        if goalieOrientation <= .. :
#            goalieHeading =  # different variable name
#        else:
#            goalieHeading = # different variable name
#        
#        predicGoaliePos = goaliePosition + goalieHeading
        
        gapRight = abs(goaliePosition[0] - rightGoalPost[0])
        gapLeft = abs(goaliePosition[0] - leftGoalPost[0])
        
        if gapRight >= gapLeft:
            xAim = rightGoalPost[0] + ballRadius + tolerance
        else:
            xAim = leftGoalPost[0] - ballRadius - tolerance
        
        return [xAim, goaliePosition[1], z]
        
    def kickingPose(self, target):
        # CHANGE TO ACTUAL POSITION IF NOT TESTING
        ballPositionX, ballPositionY = (0,-0.35)#self.getBallPose()
        distance2ball = 0.08
        deltaX = target[0] - ballPositionX
        deltaY = target[1] - ballPositionY
        if deltaY != 0:
            gamma = math.atan(deltaX/deltaY)
        else: 
            gamma = math.pi/2
        
        # transform gamma into theta orientation
        # CHECK FOR POINTS IN BETWEEN
        if ballPositionX <= target[0]:
            if ballPositionY <= target[1]:
                print '1'
                theta = - gamma
                xOffset = - distance2ball*math.sin(gamma)
                yOffset = - distance2ball*math.cos(gamma)
            else:
                # CHECK THIS ONE!
                print '2'
                theta = - gamma + math.pi
                xOffset = distance2ball*math.sin(gamma)
                yOffset = distance2ball*math.cos(gamma)
        else:
            if ballPositionY <= target[1]:
                print '3'
                theta = - gamma
                xOffset = - distance2ball*math.sin(gamma)
                yOffset = - distance2ball*math.cos(gamma)
                print xOffset
                print yOffset
            else:
                print '4'
                print gamma
                theta = - gamma + math.pi
                xOffset = distance2ball*math.sin(gamma)
                yOffset = distance2ball*math.cos(gamma)
        xPosition = ballPositionX + xOffset
        yPosition = ballPositionY + yOffset
        pose = [xPosition, yPosition, theta]
        return pose
        
    def kick(self, speed):
        self.setMotorVelocities(speed, speed)
        time.sleep(0.5)
        self.setMotorVelocities(0, 0)

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
    obj = FinalProjectProgram('Blue', 1)
    obj.run()