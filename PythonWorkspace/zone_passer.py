"""For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.

This code relies on threads in Python, more which can be learned here:
    https://pymotw.com/2/threading/"""

import base_robot
import numpy as np
import time
import threading

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
        print("%s started" % self.bot_name)
        t0 = time.time()
        while time.time() - t0 < 20:
            robotConf = self.getRobotConf(self.bot)
            self.followPath(robotConf)
        print("%s finished" % self.bot_name)

def bot1Thread():
    """thread function to handle all bot1's tasks"""
    bot = ZonePasser(color='Blue', number=1)
    bot.add_zone_destination(1)
    bot.run()
    return

def bot2Thread():
    """thread function to handle all bot2's tasks"""
    bot = ZonePasser(color='Blue', number=2)
    bot.add_zone_destination(4)
    time.sleep(1) # let the first robot go
    bot.run()
    return

def bot3Thread():
    """thread function to handle all bot3's tasks"""
    bot = ZonePasser(color='Blue', number=3)
    bot.add_zone_destination(3)
    time.sleep(2) # let the first two robots go
    bot.run()
    return

#b1 = threading.Thread(name='bot1', target=bot1Thread)
b2 = threading.Thread(name='bot2', target=bot2Thread)
b3 = threading.Thread(name='bot3', target=bot3Thread)

#b1.start()
b2.start()
b3.start()


