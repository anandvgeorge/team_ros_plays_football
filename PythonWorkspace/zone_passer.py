"""For Question 2, the Robots must do a passing drill between the 4 zones
of the playing field.
"""

import vrep
import base_robot
import numpy as np
from scipy.spatial.distance import cdist
import time
from idash import IDash
from robot_helpers import smoothPath, passPath, ThetaRange

STATE_FOLLOW_PATH = 0

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

    def add_to_path(self, path_objective):
        """ path objective is array-like, shape (3,-1) """
        path_objective = np.asarray(path_objective).reshape(3, -1)
        if self.path is not None:
            self.path = np.column_stack((
                self.path,
                path_objective
            ))
        else:
            self.path = path_objective

    def add_zone_destination(self, number):
        index = number - 1 # 0 vs 1 indexing
        self.add_to_path(self.zone_corners[:, index])

    def add_delay(self, delay):
        self.delay = delay

    def robotCode(self, state):
        """ For Robots using a cyclic executor,
        robotCode should return in a small amount of time
        (i.e. NOT be implemented with a while loop)
        """
        if state==STATE_FOLLOW_PATH:
            robotConf = self.getRobotConf(self.bot)
            self.followPath(robotConf)

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

        self.zone_centers = np.array([[CENTER_X, CENTER_X, -CENTER_X, -CENTER_X],
                                      [CENTER_Y, -CENTER_Y, CENTER_Y, -CENTER_Y]])

        self.zone_corners = np.array([[CORNER_X, CORNER_X, -CORNER_X, -CORNER_X],
                                      [CORNER_Y, -CORNER_Y, CORNER_Y, -CORNER_Y]])

        # TODO: remove me since we're not allowed to set the ball pose
        # start ball at zone 4 - 1 (0 indexed)
        ball_start = self.zone_centers[:,3]
        ball_start -= 0.05
        self.ballEngine.setBallPose(ball_start)
        self.ballEngine.update()

        self.zone_pass_plan = [4, 2, 1, 3, 4, 2] # the locations between the 5 passes

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

    def getNearestBotToZone(self, zone):
        """ Returns the index of the bot closest to the zone, even if the bot
        may not be directly inside the zone """
        bot_poses = np.vstack([bot.getRobotConf() for bot in self.bots])
        bot_poses = bot_poses[:, :2]
        assert bot_poses.shape == (len(self.bots), 2)
        zone_pose = self.zone_corners[:, zone - 1].reshape(1, 2)
        dist_from_bots = cdist(zone_pose, bot_poses)
        return np.argmin(dist_from_bots)

    def planToMoveIntoReceivingPosition(self, idx, startSmoothPathConf=None):
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
        desired_zone = self.zone_pass_plan[idx]
        final_x = self.bots[idx].zone_corners[0, desired_zone - 1]
        final_y = self.bots[idx].zone_corners[1, desired_zone - 1]
        # final theta will be facing towards center field
        _ , final_theta = ThetaRange.cart2pol(final_x, final_y)
        final_theta = ThetaRange.normalize_angle(final_theta + np.pi) # flip it towards center
        smooth_path, status = smoothPath(
            startSmoothPathConf,             # robotConf
            [final_x, final_y, final_theta], # finalConf
            r=0.01
        )
        v = 15*np.ones((1, np.size(smooth_path,1))) # 20 is default vel
        smooth_path = np.concatenate((smooth_path, v), axis=0)
        self.bots[idx].add_to_path(smooth_path)

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

                self.planToMoveIntoReceivingPosition(idx, startSmoothPathConf)
                self.bots[idx].add_delay(1*idx) # delay each by one second

            # TODO: replace system time with vrep simxTime
            t0 = time.time()
            # get the bots into position
            while time.time() - t0 < 10:
                for bot in self.bots:
                    if time.time() - t0 >= bot.delay:
                        bot.robotCode(STATE_FOLLOW_PATH)

            # follow the zone passing plan
            activezone_idx = 0
            shoot_flag = False
            plan_length = len(self.zone_pass_plan)
            executing = False
            # TODO: maybe something like below with states for all bots?
            # bot_states = [0, 1, 2] where numbers
            t1 = time.time()
            while time.time() - t1 < 60:
                self.ballEngine.update()
                activezone = self.zone_pass_plan[activezone_idx]

                if plan_length > activezone_idx + 2:
                    next_rcvzone = self.zone_pass_plan[activezone_idx + 2]
                else:
                    next_rcvzone = None

                if plan_length > activezone_idx + 1:
                    rcvzone = self.zone_pass_plan[activezone_idx + 1]
                # no more rcvers!  time to shoot
                else:
                    shoot_flag = True

                # get into position receiving player
                rcvbot_idx = self.getNearestBotToZone(rcvzone)
                # TODO: get into position receiving bot! (if not in zone)
                rcvbot = self.bots[rcvbot_idx]

                # TODO: get into position active player (if not in zone)!
                activebot_idx = self.getNearestBotToZone(activezone)
                activebot = self.bots[activebot_idx]

                # calculate path the bot has to follow
                # TODO: maybe we should continuously calculate path in case state change
                if not executing:
                    activeRobotConf = activebot.getRobotConf(activebot.bot)
                    ballRestPos = self.ballEngine.getBallPose()

                    # final ball position should be between receiving player and next receiving zone
                    xy1 = rcvbot.getRobotConf()[:2]
                    xy2 = self.zone_corners[:,next_rcvzone-1]
                    finalBallPos = self.calculateReceivingDestination(xy1, xy2, k=0.25)

                    activebot.path = passPath(activeRobotConf, ballRestPos, finalBallPos)
                    executing = True

                activebot.robotCode(STATE_FOLLOW_PATH)

                p1 = self.ballEngine.getBallPose()
                p2 = self.ballEngine.getNextRestPos()
                # TODO: maybe add "velocity" compoenent to ballEngine to know when the ball has stopped
                # ballPose has kinda achieved its expected resting position
                dist_temp = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                print "DistTemp: ", dist_temp
                if self.getClosestZone(p1) != activezone:
                    if dist_temp < 0.003:
                        # increment what is the new active zone
                        activezone_idx += 1
                        executing = False # ready to plan the next execution

                time.sleep(50*1e-3)

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