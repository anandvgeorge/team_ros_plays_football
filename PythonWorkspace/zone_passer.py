"""
For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.
"""

import base_robot
import numpy as np

class ZonePasser(base_robot.BaseRobotRunner):

    # zone_locations[0] returns zone location 1 from the diagram in the slides
    zone_locations = np.array([[0.3, 0.3, -0.3, -0.3],
                               [0.4, -0.4, 0.4, -0.4]])

    def __init__(self, *args, **kwargs):
        super(ZonePasser, self).__init__(*args, **kwargs)

    def add_zone_destination(self, number):
        index = number - 1 # 0 vs 1 indexing
        self.path = self.zone_locations[:, index].reshape(-1, 1)

    def robotCode(self):
        print self.bot_name, self.path
        while True:
            robotConf = self.getRobotConf(self.bot)
            self.followPath(robotConf)

        print "Success %s" % self.bot_name

if __name__ == '__main__':
    bot1 = ZonePasser(color='Blue', number=1)
    # FIXME: commenting out this line allows the robot to run normally
    # letting bot 2 initialize makes the robot spin in a poor direction
    bot2 = ZonePasser(color='Blue', number=2)
    # bot3 = ZonePasser(color='Blue', number=3)

    bot1.add_zone_destination(1)
    # bot2.add_zone_destination(2)
    # bot3.add_zone_destination(3)

    bot1.run()
    # bot2.run()
    # bot3.run()

