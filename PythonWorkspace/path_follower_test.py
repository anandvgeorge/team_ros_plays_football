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
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)

    def robotCode(self):
        
        self.path = np.array([[0.3, 0.3, -0.3, -0.3],
                              [0.4, -0.4, -0.4, 0.4]])
        while 1:
            robotConf = self.getRobotConf(self.bot)            
            self.followPath(robotConf)        
#                robotConf = self.getRobotConf(self.bot)
#                print('robotConf.x=%f' % robotConf[0])
#                print('robotConf.y=%f' % robotConf[1])
#                print('robotConf.theta=%f' % robotConf[2])
#                ballPos = self.getBallPose() # (x, y)
#                print('ballPos.x=%f' % ballPos[0])
#                print('ballPos.y=%f' % ballPos[1])
#                vRobot = v2Pos(robotConf, ballPos)
#                print('vRobot.x=%f' % vRobot[0])
#                print('vRobot.y=%f' % vRobot[1])
#                self.setMotorVelocities(vRobot[0], vRobot[1])
#            time.sleep(3)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1)
    obj.run()