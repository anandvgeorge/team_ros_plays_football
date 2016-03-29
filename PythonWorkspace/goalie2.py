"""
Romain test
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
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos, passPath, calculatePathTime, v2orientation
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)

    def robotCode(self):
                                
#        dash = IDash(framerate=0.1)            
#        plt.plot(self.path[0,:], self.path[1,:])  
#        dash.add(plt)
        
        #k=0.0345    # ball model: d(t) = vmot*k*(1-exp(-t/T))

#        while (self.ballEngine.getSimTime()-t)<10: #End program after 30sec
        t = time.time()
        while time.time()-t<1000:
            robotConf = self.getRobotConf(self.bot)  
            self.keepGoal(robotConf, y=0.7, goalLim=0.2)
            time.sleep(0.01)  #s
        
        self.setMotorVelocities(0,0)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1) # ip='172.29.34.63'
    obj.run()