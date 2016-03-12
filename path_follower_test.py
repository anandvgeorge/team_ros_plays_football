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
import matplotlib.pyplot as plt

from idash import IDash
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos, smoothPath
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)

    def robotCode(self):
        
        self.path = np.array([[0.3, 0.3, -0.3, -0.3],
                              [0.4, -0.4, -0.4, 0.4]])                          
#        dash = IDash(framerate=0.1)            
#        plt.plot(self.path[0,:], self.path[1,:])  
#        dash.add(plt)
        
        
        goal = (0, -0.8)
        ballPos = self.getBallPose()
        theta = math.atan2(goal[1]-ballPos[1], goal[0]-ballPos[0])   # atan2(y, x)   
        finalConf = (ballPos[0], ballPos[1], theta)   
        print('finalConf.x=%f' % finalConf[0])
        print('finalConf.y=%f' % finalConf[1])
        print('finalConf.theta=%f' % finalConf[2])
        # start, goal, r, q 
        self.path = smoothPath(self.getRobotConf(self.bot), finalConf, 0.1, 0.1)
        print self.path
        dash = IDash(framerate=0.1)
        dash.add(lambda: plt.plot(-self.path[1,:], self.path[0,:], 'b-*') and
            plt.xlim([-0.7, 0.7]) and plt.ylim([-0.7, 0.7]))
        dash.plotframe()
        cc=1
        while cc:
            robotConf = self.getRobotConf(self.bot)            
            cc = self.followPath(robotConf, 10, 0.05)        
#            robotConf = self.getRobotConf(self.bot)
#            ballPos = self.getBallPose() # (x, y)
#            vRobot = v2Pos(robotConf, ballPos)

        self.setMotorVelocities(0,0)
        time.sleep(3)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1)
    obj.run()