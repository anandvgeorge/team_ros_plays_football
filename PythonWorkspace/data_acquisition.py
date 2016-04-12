"""
Data acquisition for ball model
Romain Chiappinelli
11.03.16

using Q2.ttt and removing the red robots
""" 
import base_robot 
import time
import math
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting
import signal

from idash import IDash
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos, passPath, calculatePathTime, v2PosB
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)

    def robotCode(self):
        
        
        np.set_printoptions(linewidth=np.inf)                       
        
        cc=1
        while cc:
#            t=time.time()
            self.keepGoalP(self.getRobotConf())
#            self.driveMotor(0, 0)
#            print 'loop time'
#            print t-time.time()
#            time.sleep(0.01)
        
#            print self.getRobotConf()
        """ to measure the trajectory of the robot and ball during shoot
        need to be used with pass_axample.m matlab script"""
#        goal = [0.0, 0.0]
#        self.path, status = passPath(self.getRobotConf(self.bot), self.ballEngine.getBallPose(), goal)        
#        dash = IDash(framerate=0.1)
#        dash.add(lambda: plt.plot(-self.path[1,:], self.path[0,:], 'b-*') and
#            plt.xlim([-0.7, 0.7]) and plt.ylim([-0.7, 0.7]))
#        dash.plotframe()       
#        print 'goal='
#        print goal
#        print 'path='  
#        print self.path  
#        pr = np.array([[],[]]) #np.array([[robotConf[0]],[robotConf[1]]])   # position of robot
#        pb = np.array([[],[]]) #np.array([[ballpos[0]],[ballpos[1]]])     # position of ball
#        peg = np.array([[],[]])     # position of estimated goal
#        timesave = []   # time
#        t = self.ballEngine.getSimTime()     # time in seconds
#        while (self.ballEngine.getSimTime()-t)<20: #End program after 30sec
#            remaining = self.ballEngine.getSimTime()-t
#            timesave.append(remaining)
#            robotConf = self.getRobotConf(self.bot)            
#            self.followPath(robotConf, rb=0.05) 
#            pr = np.concatenate((pr, np.array([[robotConf[0]],[robotConf[1]]])), axis=1)
#            ballpos = self.ballEngine.getBallPose()
#            pb = np.concatenate((pb, np.array([[ballpos[0]],[ballpos[1]]])), axis=1)
#            self.ballEngine.update()
#            ballpos = self.ballEngine.getNextRestPos()
#            print 'ball valocity'
#            print self.ballEngine.getVeloctiy()
#            peg = np.concatenate((peg, np.array([[ballpos[0]],[ballpos[1]]])), axis=1)
#            
#        self.setMotorVelocities(0,0)        
#
#        print 'robotPos='
#        print pr
#        print 'ballPos='  
#        print pb
#        print 'goalEstim='
#        print peg
        
        """ to measure distance of the ball after shoot
            need to be used with ballModel.m matlab script"""
#        veolcity = 13   # velocity of the robot
#        #T = 0.01   # sampling time in ms
#        pf = self.ballEngine.getBallPose()
#        # start, goal, r, q 
#        d = []
#        robotConf0 = self.getRobotConf(self.bot)  
#        dr = []
#        timesave = []
#        t = self.ballEngine.getSimTime()    # time in seconds
#        while (self.ballEngine.getSimTime()-t)<30: #End program after 30sec
#            remaining = self.ballEngine.getSimTime()-t
#            timesave.append(remaining)
#            #print "Time Remaining=", remaining    
#            robotConf = self.getRobotConf(self.bot)            
#            v = v2Pos(robotConf, pf, veolcity)
#            self.setMotorVelocities(v[0], v[1])
#            ballPos = self.ballEngine.getBallPose()
#            d1 = ((ballPos[0]-pf[0])**2 + (ballPos[1]-pf[1])**2)**0.5
#            d.append(d1)  
#            d1 = ((robotConf[0]-robotConf0[0])**2 + (robotConf[1]-robotConf0[1])**2)**0.5
#            dr.append(d1) 
#            
#        self.setMotorVelocities(0,0)
#
#        print 'vmotors=%f' %veolcity
#        print 'time'
#        print timesave
#        print 'ball distance'
#        print d
#        print 'robot distance'
#        print dr        


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1)
    obj.run()