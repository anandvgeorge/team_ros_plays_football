"""
Path follower test
Romain Chiappinelli
04.03.16
""" 
import base_robot 
import time
import math
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting
import signal

from idash import IDash
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos, passPath, calculatePathTime
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)

    def robotCode(self):
        
        self.path = np.array([[0.2, 0.2, -0.2, -0.2],
                              [0.4, -0.4, -0.4, 0.4],
                              [15, 15, 15, 15]])                          
#        dash = IDash(framerate=0.1)            
#        plt.plot(self.path[0,:], self.path[1,:])  
#        dash.add(plt)
        
        #k=0.0345    # ball model: d(t) = vmot*k*(1-exp(-t/T))

        goal = (0.0, 0.0)
        self.path, status = passPath(self.getRobotConf(self.bot), self.ballEngine.getBallPose(), goal)
        print self.path
        
        dash = IDash(framerate=0.1)
        dash.add(lambda: plt.plot(-self.path[1,:], self.path[0,:], 'b-*') and
            plt.xlim([-0.7, 0.7]) and plt.ylim([-0.7, 0.7]))
        dash.plotframe()       
        print 'estimated time path'
        print calculatePathTime(self.path)
        t = self.ballEngine.getSimTime()     # time in seconds
#        while (self.ballEngine.getSimTime()-t)<10: #End program after 30sec
        cc=1
        while cc:
            robotConf = self.getRobotConf(self.bot)            
            cc = self.followPath(robotConf, status, rb=0.05) 
            self.ballEngine.update()
            nextBallPos = self.ballEngine.getNextRestPos()
#            robotConf = self.getRobotConf(self.bot)
#            ballPos = self.ballEngine.getBallPose() # (x, y)
#            vRobot = v2Pos(robotConf, ballPos)
        
        self.setMotorVelocities(0,0)
        print 'real time path'
        print (self.ballEngine.getSimTime()-t)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1) # ip='172.29.34.63'
    obj.run()