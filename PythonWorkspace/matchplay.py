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
from robot_helpers import smoothPath, passPath, ThetaRange, prox_sens_read

class Player(base_robot.BaseRobotRunner):
    def __init__(self, *args, **kwargs):
        super(Player, self).__init__(*args, **kwargs)

    def robotCode(self):
        out = prox_sens_read(self.clientID, self.proxSensors)
        objectDetected = np.zeros(4)
        for i, sens in enumerate(out):
            # print sens['detectionState']
            if sens['detectionState'] == 1:
                objectDetected[i] = 1
        return objectDetected

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
                for bot in self.bots:
                    print bot.robotCode()

if __name__ == '__main__':
    master = Master(ip='127.0.0.1')
    bot2 = Player(color='Blue', number=2, clientID=master.clientID)
    master.addRobot(bot2)
    master.run()