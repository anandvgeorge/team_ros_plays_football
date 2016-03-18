"""For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.
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
    """After doing part A of Question 2, the ball will already be
    placed in Zone 4; so let's transport it there now"""
    
    def __init__(self, *args, **kwargs):
        super(ZonePasserMasterCyclic, self).__init__(*args, **kwargs)
        self.activezones = []
        self.receivingzones = []
        self.activeplayer = None
        self.receivingplayer = None
        self.zones = {( 1,  1): 1,
                      ( 1, -1): 2,
                      (-1,  1): 3,
                      (-1, -1): 4}

        self.zone_locations = np.array([[0.3, 0.3, -0.3, -0.3],
                                        [0.4, -0.4, 0.4, -0.4],])

        # TODO: remove me since we're not allowed to set the ball pose
        # start ball at zone 4 - 1 (0 indexed)
        ball_start = self.zone_locations[:,3]
        ball_start -= 0.05
        self.ballEngine.setBallPose(ball_start)

    def getClosestZone(self, pose):
        """ get zone which the current pose is closest to. This pose could
        be a ball, an opposing player, etc. """
        sgn_x = np.sign(pose[0])
        sgn_y = np.sign(pose[1])
        return self.zones[(sgn_x, sgn_y)]

    def getBotInZone(self, zone):
        """ Returns the index of the bot which is inside the zone """
        bot_zones = np.zeros(len(self.bots))
        print("Entered getBotInZone")
        for bot_idx, bot in enumerate(self.bots):
            bot_zones[bot_idx] = self.getClosestZone(bot.getRobotConf(bot.bot))
        return np.where(bot_zones == zone)
    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
            self.ballpose = self.ballEngine.getBallPose() # get initial ball pose
            # first active zone is where the ball starts           
            self.activezones.append(self.getClosestZone(self.ballpose))
            activezone_idx = 0            

            self.activeplayer = self.getBotInZone(self.activezones[activezone_idx])
            print self.activeplayer

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