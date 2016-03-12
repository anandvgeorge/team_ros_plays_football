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

        veolcity = 25   # velocity of the robot
        #T = 0.01   # sampling time in ms
    
        pf = self.getBallPose()
        # start, goal, r, q 

        d = []
        robotConf0 = self.getRobotConf(self.bot)  
        dr = []
        timesave = []
        t = time.time()     # time in seconds
        while (time.time()-t)<15: #End program after 30sec
            remaining = time.time()-t
            timesave.append(remaining)
            #print "Time Remaining=", remaining    
            robotConf = self.getRobotConf(self.bot)            
            v = v2Pos(robotConf, pf, veolcity)
            self.setMotorVelocities(v[0], v[1])
            ballPos = self.getBallPose()
            d1 = ((ballPos[0]-pf[0])**2 + (ballPos[1]-pf[1])**2)**0.5
            d.append(d1)  
            d1 = ((robotConf[0]-robotConf0[0])**2 + (robotConf[1]-robotConf0[1])**2)**0.5
            dr.append(d1) 
            
        self.setMotorVelocities(0,0)

        print 'vmotors=%f' %veolcity
        print 'time'
        print timesave
        print 'ball distance'
        print d
        print 'robot distance'
        print dr        


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1)
    obj.run()