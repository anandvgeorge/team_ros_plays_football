"""Cyclic implementation 
"""

import vrep
import base_robot
import numpy as np
import time

class ZonePasser(base_robot.BaseRobotRunner):

    # zone_locations[0] returns zone location 1 from the diagram in the slides
    zone_locations = np.array([[0.3, 0.3, -0.3, -0.3],
                               [0.4, -0.4, 0.4, -0.4],
                               [20, 20, 20, 20]])

    def __init__(self, *args, **kwargs):
        super(ZonePasser, self).__init__(*args, **kwargs)
        self.delay = 0

    def add_zone_destination(self, number):
        index = number - 1 # 0 vs 1 indexing
        self.path = self.zone_locations[:, index].reshape(-1, 1)

    def add_delay(self, delay):
        self.delay = delay

    def robotCode(self):
        print("%s started" % self.bot_name)
        t0 = time.time()
        time.sleep(self.delay)
        while time.time() - t0 < 20:
            robotConf = self.getRobotConf(self.bot)
            self.followPath(robotConf)
        print("%s finished" % self.bot_name)

class ZonePasserCyclic(ZonePasser):

    def __init__(self, *args, **kwargs):
        super(ZonePasserCyclic, self).__init__(*args, **kwargs)

    def robotCode(self):
        """ For Robots using a cyclic executor,
        robotCode should return in a small amount of time
        (i.e. NOT be implemented with a while loop)
        """
        robotConf = self.getRobotConf(self.bot)
        self.followPath(robotConf)

class ZonePasserMaster(base_robot.MultiRobotRunner):

    def __init__(self, *args, **kwargs):
        super(ZonePasserMaster, self).__init__(*args, **kwargs)

class ZonePasserMasterCyclic(base_robot.MultiRobotCyclicExecutor):

    def __init__(self, *args, **kwargs):
        super(ZonePasserMasterCyclic, self).__init__(*args, **kwargs)

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
            t0 = time.time()
            while time.time() - t0 < 10:
                for bot in self.bots:
                    if time.time() - t0 >= bot.delay:
                        bot.robotCode()

        self.clean_exit()

runner = ZonePasserMasterCyclic(ip='127.0.0.1')

print "runnerClientID", runner.clientID
bot1 = ZonePasserCyclic(color='Blue', number=1, clientID=runner.clientID)
bot1.add_zone_destination(1)
runner.addRobot(bot1)

bot2 = ZonePasserCyclic(color='Blue', number=2, clientID=runner.clientID)
bot2.add_zone_destination(4)
bot2.add_delay(1)
runner.addRobot(bot2)

bot3 = ZonePasserCyclic(color='Blue', number=3, clientID=runner.clientID)
bot3.add_zone_destination(3)
bot3.add_delay(2)
runner.addRobot(bot3)

runner.run()