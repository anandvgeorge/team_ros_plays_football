"""For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.
"""

import vrep
import base_robot
import numpy as np
from scipy.spatial.distance import cdist
import time
import matplotlib.pyplot as plt
from idash import IDash
from robot_helpers import smoothPath, passPath, ThetaRange

STATE_READY_POS = 0
STATE_PASS = 1
STATE_SHOOT = 2

CORNER_X = 0.385
CORNER_Y = 0.55

CENTER_X = 0.2
CENTER_Y = 0.45

class ZonePasserCyclic(base_robot.BaseRobotRunner):

    # zone_corners[0] returns zone location 1 from the diagram in the slides
    zone_corners = np.array([[CORNER_X, CORNER_X, -CORNER_X, -CORNER_X],
                             [CORNER_Y, -CORNER_Y, CORNER_Y, -CORNER_Y],
                             [20, 20, 20, 20]])

    def __init__(self, *args, **kwargs):
        super(ZonePasserCyclic, self).__init__(*args, **kwargs)
        self.delay = 0
        self.path = None



    def add_zone_destination(self, number):
        index = number - 1 # 0 vs 1 indexing
        self.add_to_path(self.zone_corners[:, index])

    def add_delay(self, delay):
        self.delay = delay

    def robotCode(self, rb=0.05, k=2.5):
        """ For Robots using a cyclic executor,
        robotCode should return in a small amount of time
        (i.e. NOT be implemented with a while loop)
        """
        robotConf = self.getRobotConf(self.bot)
        return self.followPath(robotConf, rb=rb, k=k)

class ZonePasserMasterCyclic(base_robot.MultiRobotCyclicExecutor):
    """After doing part A of Question 2, the ball will already be
    placed in Zone 4; so let's transport it there now"""


    def __init__(self, *args, **kwargs):
        super(ZonePasserMasterCyclic, self).__init__(*args, **kwargs)
        self.idash = IDash(0.005)
        self.activezones = []
        self.receivingzones = []
        self.activeplayer = None
        self.receivingplayer = None
        self.zones = {( 1,  1): 1,
                      ( 1, -1): 2,
                      (-1,  1): 3,
                      (-1, -1): 4}

        self.zone_centers = np.array([[CENTER_X, CENTER_X, -CENTER_X, -CENTER_X],
                                      [CENTER_Y, -CENTER_Y, CENTER_Y, -CENTER_Y]])

        self.zone_corners = np.array([[CORNER_X, CORNER_X, -CORNER_X, -CORNER_X],
                                      [CORNER_Y, -CORNER_Y, CORNER_Y, -CORNER_Y]])

        # TODO: remove me since we're not allowed to set the ball pose
        # start ball at zone 4 - 1 (0 indexed)
        ball_start = self.zone_centers[:,3]
        ball_start[1] -= 0.05 # move closer to players center, but further distance q from player
        self.ballEngine.setBallPose(ball_start)
        self.ballEngine.update()

        # FIXME: Hardcoded is good for drill, not good for game!
        self.zone_pass_plan = [4, 2, 1, 3, 4, 2] # the locations between the 5 passes
        self.activebot_idx_plan = [0, 1, 2, 0, 1, 2] # which robot should take the active zone during the zone pass plan?

    def getClosestZone(self, pose):
        """ get zone which the current pose is closest to. This pose could
        be a ball, an opposing player, etc. """
        pose = np.array(pose[:2]) # x-y only
        assert pose.size == 2 # pose should be 2-D
        dist_from_zones = cdist(np.expand_dims(pose, axis=0), self.zone_corners.T)
        return np.argmin(dist_from_zones) + 1 # zones are 1-indexed

    def getBotInZone(self, zone):
        """
        Not using now
        Returns the index of the bot which is inside the zone, otherwise None
        if there is no bot inside this zone"""
        bot_zones = np.zeros(len(self.bots))
        for bot_idx, bot in enumerate(self.bots):
            bot_zones[bot_idx] = self.getClosestZone(bot.getRobotConf(bot.bot))
        # we use [0][0] since return of np.where looks like (array([2]),)
        zone_as_array = np.where(bot_zones == zone)[0]
        if zone_as_array.size == 0:
            return None
        else:
            return zone_as_array[0] # the first, if there are multiple in that zone

    def getNearestBotToZone(self, zone, bot_to_ignore=None):
        """ Returns the index of the bot closest to the zone, even if the bot
        may not be directly inside the zone """
        bot_poses = np.vstack([bot.getRobotConf() for bot in self.bots])
        bot_poses = bot_poses[:, :2]
        assert bot_poses.shape == (len(self.bots), 2)
        zone_pose = self.zone_corners[:, zone - 1].reshape(1, 2)
        dist_from_bots = cdist(zone_pose, bot_poses, 'cityblock')
        if bot_to_ignore is None:
            return np.argmin(dist_from_bots)
        else: # the closest bot might be active, in which case we should return next closest
            sortedargs = np.argsort(dist_from_bots.flatten())
            if sortedargs[0] == bot_to_ignore: # first closest is active, not valid to be receive
                return sortedargs[1] # return second closest
            else:
                return sortedargs[0]

    def planToMoveIntoReceivingPosition(self, idx, rcvzone=None, startSmoothPathConf=None, vel=15, r=0.01):
        """ Move into the corner, facing center
        Parameters
        ----------
        idx: integer
            index of the robot (in self.bots)

        startSmoothPathConf: array-like, shape (3,) or None
            the start configuration of the robot to calculate the smooth path.
            Pass this if you are calculating a path from a future point.
            If None, then use current robot configuration.
        """
        if startSmoothPathConf is None:
            startSmoothPathConf = self.bots[idx].getRobotConf()
        if rcvzone is None:
            rcvzone = self.zone_pass_plan[idx]
        final_x = self.bots[idx].zone_corners[0, rcvzone - 1]
        final_y = self.bots[idx].zone_corners[1, rcvzone - 1]
        # final theta will be facing towards center field
        _ , final_theta = ThetaRange.cart2pol(final_x, final_y)
        final_theta = ThetaRange.normalize_angle(final_theta + np.pi) # flip it towards center
        smooth_path, status = smoothPath(
            startSmoothPathConf,             # robotConf
            [final_x, final_y, final_theta], # finalConf
            r=r
        )
        v = vel*np.ones((1, np.size(smooth_path,1)))
        smooth_path = np.concatenate((smooth_path, v), axis=0)
        if self.bots[idx].path is None:
            self.bots[idx].add_to_path(smooth_path)
        elif self.bots[idx].path.shape[1] == 1:
            self.bots[idx].add_to_path(smooth_path)
        else:
            self.bots[idx].path = smooth_path

    def calculateReceivingDestination(self, xy1, xy2, k=0.2):
        """ receiving BallPos should be between rcvbot and next passing zone
        Parameters
        ----------
        xy1: array-like, shape (2,)
            x-y position of receiving bot
        xy2: array-like, shape (2,)
            x-y position of next receiving zone
        k: float, between 0-1
            multiplier to determine how much in between the rcv and nextrcv
        """
        x1, y1 = xy1
        x2, y2 = xy2
        theta = np.arctan2(y1 - y2, x1 - x2)
        hyp = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        rcvBallPos = [x1 - k*hyp*np.cos(theta), y1 - k*hyp*np.sin(theta)]
        return rcvBallPos

    def calculateShootingDestination(self):
        # TODO: Intelligently use position of moving opponenet goalie
        posToAim = [0, -0.75] # in the center of the goal
        return posToAim

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)

            # set them up in the first 3 zones
            for idx in range(len(self.bots)):
                if self.zone_pass_plan[idx] == 2:
                    # zone 2 has some robots blocking, so navigate to zone 2
                    # in a couple of steps
                    # TOOD: may want to do this less hardcoded, more like planned path with obstacle avoidance
                    # step 1. get to midfield passing the opposing team
                    x_coord_zone = 0.3
                    self.bots[idx].add_to_path([x_coord_zone, 0, 20])
                    startSmoothPathConf = [x_coord_zone, 0, -np.pi/2]
                else:
                    startSmoothPathConf = self.bots[idx].getRobotConf(self.bots[idx].bot)

                print("Plannning To Receive, 1st Time")
                self.planToMoveIntoReceivingPosition(idx, startSmoothPathConf=startSmoothPathConf, vel=10)
                self.bots[idx].add_delay(1*idx) # delay each by one second

            # TODO: replace system time with vrep simxTime
            t0 = time.time()
            # get the bots into position
            while time.time() - t0 < 15:
                for bot in self.bots:
                    if time.time() - t0 >= bot.delay:
                        bot.robotCode()

            # follow the zone passing plan
            activezone_idx = 0
            shoot_flag = False
            plan_length = len(self.zone_pass_plan)
            executing = [False] * len(self.bots)
            # TODO: maybe something like below with states for all bots?
            bot_states = [STATE_READY_POS] * len(self.bots)
            t1 = time.time()
            while time.time() - t1 < 180:
                self.ballEngine.update()

                activezone = self.zone_pass_plan[activezone_idx]
                activebot_idx = self.activebot_idx_plan[activezone_idx]
                activebot = self.bots[activebot_idx]
                if plan_length > activezone_idx + 1:
                    rcvzone = self.zone_pass_plan[activezone_idx + 1]
                    rcvbot_idx = self.activebot_idx_plan[activezone_idx + 1]
                    rcvbot = self.bots[rcvbot_idx]
                else:
                    rcvzone = None
                    rcvbot_idx = None
                    rcvbot = None
                    shoot_flag = True
                if plan_length > activezone_idx + 2:
                    next_rcvzone = self.zone_pass_plan[activezone_idx + 2]
                else:
                    next_rcvzone = None


                def vizZones():
                    zones = np.zeros(4)
                    zones[activezone-1] = 0.5
                    if rcvzone:
                        zones[rcvzone-1] = 1.0
                    plt.imshow(zones.reshape(2,2), interpolation='nearest')
                    plt.title('Red = RCV, Green = Active')
                self.idash.add(vizZones)

                # -- STATE MACHINE UPDATE PARAMETERS
                if shoot_flag:
                    bot_states[activebot_idx] = STATE_SHOOT
                else:
                    bot_states[activebot_idx] = STATE_PASS
                if rcvbot_idx:
                    bot_states[rcvbot_idx] = STATE_READY_POS

                # -- STATE MACHINE EXECUTE
                    if bot_states[rcvbot_idx] == STATE_READY_POS:
                        rcvp1 = np.array(rcvbot.getRobotConf()[:2])
                        rcvp2 = self.zone_corners[:, rcvzone - 1]
                        # not yet in position
                        print "In receiving Position? ", cdist(rcvp1.reshape(1,2), rcvp2.reshape(1,2))[0]
                        if cdist(rcvp1.reshape(1,2), rcvp2.reshape(1,2))[0] > 0.001: # radius buffer
                            if not executing[rcvbot_idx]:
                                if next_rcvzone:
                                    xy2 = self.zone_centers[:,next_rcvzone-1]
                                else:
                                    xy2 = [0, -1.0]
                                print("Plannning To Receive, 2nd Time")
                                self.planToMoveIntoReceivingPosition(rcvbot_idx, rcvzone, vel=10)
                                rcvbot.prunePath()
                                executing[rcvbot_idx] = True
                            rcvbot.robotCode(rb=0.05)
                        else:
                            executing[rcvbot_idx] = False

                if bot_states[activebot_idx] == STATE_PASS:
                    if not executing[activebot_idx]:
                        activeRobotConf = self.bots[activebot_idx].getRobotConf()
                        p0 = self.ballEngine.getBallPose()
                        xy1 = self.zone_centers[:,rcvzone-1]
                        if next_rcvzone:
                            xy2 = self.zone_centers[:,next_rcvzone-1]
                        else:
                            xy2 = [0, -0.75]
                        finalBallPos = self.calculateReceivingDestination(xy1, xy2, k=0.05)
                        self.bots[activebot_idx].path = passPath(activeRobotConf, p0, finalBallPos, vmax=10, vr=7, kq=0.0035)
                        self.bots[activebot_idx].prunePath()
                        p2 = self.ballEngine.getNextRestPos()
                        # FIXME: if the path produced is invalid, i.e some part of it is off the field and invalid
                        #        prune that part of the path to make it valid
                        # FIXME: if path is not long enough
                        #        backup to give more room. or bump the ball and get more room.

                        def vizBots():
                            actx, acty = self.bots[activebot_idx].getRobotConf()[:2]
                            rcvx, rcvy = rcvbot.getRobotConf()[:2]
                            plt.hold('on')
                            plt.plot(-acty, actx, 'g+')
                            plt.plot(-self.bots[activebot_idx].path[1,:], self.bots[activebot_idx].path[0,:], 'g.')
                            plt.plot(-rcvy, rcvx, 'r+')
                            plt.plot(-rcvbot.path[1,:], rcvbot.path[0,:], 'r.')
                            plt.plot(-finalBallPos[1], finalBallPos[0], 'bo')
                            plt.plot(-rcvp1[1], rcvp1[0], 'mx')
                            plt.plot(-rcvp2[1], rcvp2[0], 'kx')
                            plt.xlim([-0.75, 0.75]) # y axis in the field
                            plt.ylim([-0.5, 0.5]) # x axis in the field
                            plt.title('Red = RCV, Green = Active')
                            plt.xlabel('active path length: {}'.format(self.bots[activebot_idx].path.shape[1]))
                        self.idash.add(vizBots)
                        executing[activebot_idx] = True
                    self.bots[activebot_idx].robotCode(rb=0.05)

                    p1 = self.ballEngine.getBallPose()
                    p3 = self.ballEngine.getNextRestPos()
                    dist_from_start = np.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)
                    dist_from_goal = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                    velocity_measure = np.sqrt((p1[0] - p3[0])**2 + (p1[1] - p3[1])**2)
                    closest_zone = self.getClosestZone(p1)

                    if dist_from_start > 0.01: # the ball has been touched
                         if velocity_measure < 0.003: # wait til velocity reaches zero
                         # if dist_from_goal < 0.1: # wait til the ball has entered the predicted zone
                            executing = [False] * len(self.bots) # everyone has new roles, plan next pass
                            if closest_zone == rcvzone: # success
                                # increment what is the new active zone
                                activezone_idx += 1

                if bot_states[activebot_idx] == STATE_SHOOT:
                    if not executing[activebot_idx]:
                        activeRobotConf = self.bots[activebot_idx].getRobotConf(self.bots[activebot_idx].bot)
                        ballRestPos = self.ballEngine.getBallPose()
                        finalBallPos = self.calculateShootingDestination()
                        activebot.path = passPath(activeRobotConf, ballRestPos, finalBallPos, vmax=10, vr=7, kq=0.005)
                        executing[activebot_idx] = True
                    self.bots[activebot_idx].robotCode()

                self.idash.plotframe()
                # time.sleep(50*1e-3)

        self.clean_exit()

runner = ZonePasserMasterCyclic(ip='127.0.0.1')

print "runnerClientID", runner.clientID
bot1 = ZonePasserCyclic(color='Blue', number=1, clientID=runner.clientID)
#bot1.add_zone_destination(1)
runner.addRobot(bot1)

bot2 = ZonePasserCyclic(color='Blue', number=2, clientID=runner.clientID)
#bot2.add_zone_destination(4)
#bot2.add_delay(1)
runner.addRobot(bot2)

bot3 = ZonePasserCyclic(color='Blue', number=3, clientID=runner.clientID)
#bot3.add_zone_destination(3)
#bot3.add_delay(2)
runner.addRobot(bot3)

runner.run()