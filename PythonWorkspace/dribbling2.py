"""

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
        self.idash = IDash(0.005)

    def robotCode(self):
        #waypoint = [(0.2, 0.4), (0, 0.4), (-0.2, 0.4),(-0.2, 0.2), (-0.2, 0),(0, 0), (0.2, 0),(0.2, -0.2), (0.2, -0.4),(0, -0.4), (-0.2, -0.4)]
        waypoint = [(0, 0.4), (-0.2, 0.4),(-0.2, 0.2), (-0.2, 0),(0, 0), (0.2, 0),(0.2, -0.2), (0.2, -0.4),
                        (0, -0.4),(-0.05, -0.4), (-0.1, -0.4), (-0.15, -0.4), (-0.2, -0.4), (-0.2, -0.4)]
        # waypoint = []
        # r = 0.2
        # increments = 5
        # xCenter = 0
        # yCenter = 0.2
        # for i in range(increments-1):
        #     # circle center: (0, 0.2)
        #     x = r*math.sin(i*math.pi/increments) + xCenter
        #     y = r*math.cos(i*math.pi/increments) + yCenter
        #     waypoint.append((x,y))
        # xCenter = 0
        # yCenter = -0.2
        # for i in range(increments):
        #     # circle center: (0, -0.2)
        #     x = -r*math.sin(i*math.pi/increments) + xCenter
        #     y = r*math.cos(i*math.pi/increments) + yCenter
        #     waypoint.append((x,y))
        # for i in range(3):
        #     waypoint.append((-0.2/4*i,-0.4))
        
        #k=0.0345    # ball model: d(t) = vmot*k*(1-exp(-t/T))

        for i in range(len(waypoint)):
            p0 = self.ballEngine.getBallPose()
            curr_bot_pose = list(self.getRobotConf(self.bot))
            curr_bot_pose[1] -= 0.05
            self.path, status = passPath(curr_bot_pose, p0, waypoint[i],vmax=10,vr=7, kq= 0.0035)
            
            print self.path
            
#            dash = IDash(framerate=0.1)
#            dash.add(lambda: plt.plot(-self.path[1,:], self.path[0,:], 'b-*') and
#                plt.xlim([-0.7, 0.7]) and plt.ylim([-0.7, 0.7]))
#            dash.plotframe()       
            print 'estimated time path'
            # print calculatePathTime(self.path)
            t = self.ballEngine.getSimTime()
            t = self.ballEngine.getSimTime()     # time in seconds
            while (self.ballEngine.getSimTime()-t)<10: #End program after 30sec
    #        cc=1
    #        while cc:
                def vizBots():
                    actx, acty = self.getRobotConf()[:2]
                    ballx, bally = self.ballEngine.getBallPose()[:2]
                    plt.hold('on')
                    plt.plot(-waypoint[i][1], waypoint[i][0], 'k*')
                    plt.plot(-bally, ballx, 'b.')
                    plt.plot(-acty, actx, 'g+')
                    plt.plot(-self.path[1,:], self.path[0,:], 'r.')
                    plt.xlim([-0.9, 0.9]) # y axis in the field
                    plt.ylim([-0.65, 0.65]) # x axis in the field
                    plt.xlabel('active path length: {}'.format(self.path.shape[1]))
                self.idash.add(vizBots)
                
                robotConf = self.getRobotConf(self.bot)            
                cc = self.followPath(robotConf, rb=0.05) 
                self.ballEngine.update()
                
                
                p1 = self.ballEngine.getBallPose()
                p3 = self.ballEngine.getNextRestPos()
                dist_from_start = np.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)
                velocity_measure = np.sqrt((p1[0] - p3[0])**2 + (p1[1] - p3[1])**2)
                #self.idash.plotframe()
                if dist_from_start > 0.01: # the ball has been touched
                    if velocity_measure < 0.003:
                        break# wait til velocity reaches zero
    #            robotConf = self.getRobotConf(self.bot)
    #            ballPos = self.ballEngine.getBallPose() # (x, y)
    #            vRobot = v2Pos(robotConf, ballPos)
                
        self.setMotorVelocities(0,0)
        print 'real time path'
        print (self.ballEngine.getSimTime()-t)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1) # ip='172.29.34.63'
    obj.run()