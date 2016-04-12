"""matchplay.py
play ball!
TODO: say this same phrase in a football context, in the german language haha.
"""
import vrep
import base_robot
import numpy as np
from scipy.spatial.distance import cdist
import time
import matplotlib.pyplot as plt
from idash import IDash
from robot_helpers import smoothPath, passPath, ThetaRange, v2PosB

class Player(base_robot.BaseRobotRunner):
    def __init__(self, *args, **kwargs):
        """init for each robot"""
        super(Player, self).__init__(*args, **kwargs)

    def robotCode(self):
        """inner while loop for each robots"""
        objectDetected, objectDistances = self.senseObstacles()
        return objectDetected, objectDistances
    
class Attacker(base_robot.BaseRobotRunner):
    def __init__(self, goal_pos=7.5, *args, **kwargs):
        """init for Attacker robot"""
        super(Attacker, self).__init__(*args, **kwargs)
        self.goal_p=goal_pos
        goal = (-1.5+3*np.random.rand(), self.goal_p)
        self.path, self.status = passPath(self.getRobotConf(self.bot), self.ballEngine.getBallPose(), goal, kick=True)

    def robotCode(self):
        """inner while loop for Attacker robot"""
        cc=self.followPath(self.getRobotConf(self.bot), self.status, rb=0.05)
        if cc==0:
            goal = (-1.5+3*np.random.rand(), self.goal_p)
            self.path, self.status = passPath(self.getRobotConf(self.bot), self.ballEngine.getBallPose(), goal, kick=True, vKick=25)
            self.prunePath(xlim=0.45, ylim=0.7)

class Goalie(base_robot.BaseRobotRunner):
    def __init__(self, goal_pos=0.72, *args, **kwargs):
        """init for Goalie robot"""
        super(Goalie, self).__init__(*args, **kwargs)
        self.goal=goal_pos

    def robotCode(self):
        """inner while loop for Goalie robot"""
        self.keepGoal2(self.getRobotConf(self.bot), self.goal)
        
class Dumb(base_robot.BaseRobotRunner):
    def __init__(self, v=30, *args, **kwargs):
        """init for Dumb robot"""
        super(Dumb, self).__init__(*args, **kwargs)
        self.vmax=v
        
    def robotCode(self):
        """inner while loop for Dumb robot"""
        vRobot = v2PosB(self.getRobotConf(self.bot), self.ballEngine.getBallPose(),self.vmax)
        self.setMotorVelocities(vRobot[0], vRobot[1])

class Master(base_robot.MultiRobotCyclicExecutor):
    def __init__(self, *args, **kwargs):
        super(Master, self).__init__(*args, **kwargs)

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)

            # in zone passer, it was
            # get to the 3 starting positions
            # while game is still going...
            # do things in a while loop

            while True:
                t=time.time()
                for bot in self.bots:
                    bot.robotCode()
                print 'loop time'
                print time.time()-t
#                self.bots[0].robotCode()
#                self.bots[1].robotCode()
#                self.bots[2].robotCode()
#                time.sleep(1)

if __name__ == '__main__':
#    master = Master(ip='192.168.0.100', port=19999) # 127.0.0.1
    master = Master(ip='127.0.0.1')
    master.addRobot(Goalie(goal_pos=0.72, color='Blue', number=1, clientID=master.clientID))
#    master.addRobot(Attacker(goal_pos= 0.72, color='Red', number=2, clientID=master.clientID))
#    master.addRobot(Attacker(goal_pos= 0.72, color='Red', number=3, clientID=master.clientID))
#    master.addRobot(Dumb(v=25, color='Red', number=2, clientID=master.clientID))
#    master.addRobot(Dumb(v=26, color='Red', number=3, clientID=master.clientID))
#    master.addRobot(Goalie(goal_pos= 0.72, color='Blue', number=1, clientID=master.clientID))
#    master.addRobot(Dumb(v=24, color='Blue', number=2, clientID=master.clientID))
#    master.addRobot(Dumb(v=31, color='Blue', number=3, clientID=master.clientID))
#    master.addRobot(Attacker(goal_pos=-0.72, color='Blue', number=2, clientID=master.clientID))
#    master.addRobot(Attacker(goal_pos=-0.72, color='Blue', number=3, clientID=master.clientID))
#    master.addRobot(Goalie(goal_pos= 0.72, color='Blue', number=1, clientID=master.clientID))
#    master.addRobot(Attacker(goal_pos= 0.72, color='Red', number=1, clientID=master.clientID))      
    master.run()