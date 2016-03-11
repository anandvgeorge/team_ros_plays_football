"""For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.
"""

import base_robot
import numpy as np
import time

class ZonePasser(base_robot.BaseRobotRunner):

    # zone_locations[0] returns zone location 1 from the diagram in the slides
    zone_locations = np.array([[0.3, 0.3, -0.3, -0.3],
                               [0.4, -0.4, 0.4, -0.4]])

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

class ZonePasserMaster(base_robot.MultiRobotRunner):

    def __init__(self, *args, **kwargs):
        super(ZonePasserMaster, self).__init__(*args, **kwargs)

runner = ZonePasserMaster(ip='127.0.0.1')

print "runnerClientID", runner.clientID
bot1 = ZonePasser(color='Blue', number=1, clientID=runner.clientID)
bot1.add_zone_destination(1)
runner.addRobot(bot1)

bot2 = ZonePasser(color='Blue', number=2, clientID=runner.clientID)
bot2.add_zone_destination(4)
bot2.add_delay(1)
runner.addRobot(bot2)

bot3 = ZonePasser(color='Blue', number=3, clientID=runner.clientID)
bot3.add_zone_destination(3)
bot3.add_delay(2)
runner.addRobot(bot3)

runner.run()