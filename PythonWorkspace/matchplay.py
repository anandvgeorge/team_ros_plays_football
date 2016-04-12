"""matchplay.py
ANSTOSS!!!!
"""
import vrep
import base_robot
import numpy as np
import time
import matplotlib.pyplot as plt
from idash import IDash
from robot_helpers import (passPath, v2PosB, aim)



class Player(base_robot.BaseRobotRunner):
    def __init__(self, *args, **kwargs):
        super(Player, self).__init__(*args, **kwargs)
        self.executing = False
        self.conf2 = np.array(self.getRobotConf())
        self.conf1 = self.conf2 + np.random.randn(3)*0.1
        self.conf0 = self.conf2 + np.random.randn(3)*0.1
        self.time_started_2b_dumb = 0
        self.p0 = self.ballEngine.getBallPose()

        self.attackerInit()
        self.goalieInit()

    def attackerInit(self):
        # set random goal position to shoot
        self.oppGoal = [-1.5+3*np.random.rand(), -7.5]
        if self.color == 'Red':
            self.oppGoal[1] *= -1

        self.passivePos = np.array([0, -0.2, np.pi/2])
        if self.color == 'Red':
            self.passivePos[1:] *= -1

        # Came from midfielder
        # self.passivePos = np.array([0, 0.1, -np.pi/2])
        # if self.color == 'Red':
        #     self.passivePos[1:] *= -1

    def goalieInit(self):
        self.ourGoal=0.72 # FIXME: hardcoded bad!
        if self.color == 'Red':
            self.ourGoal *= -1

    def robotCode(self, role, *args, **kwargs):
        if role == 'goalie':
            self.goalie_robotCode(*args, **kwargs)
        elif role == 'midfielder':
            self.midfielder_robotCode(*args, **kwargs)
        elif role == 'attacker':
            self.attacker_robotCode(*args, **kwargs)
        elif role == 'dumb':
            self.dumb_robotCode(*args, **kwargs)

    def goalie_robotCode(self, *args, **kwargs):
        """inner while loop for Goalie robot"""
        self.keepGoalP(self.getRobotConf(self.bot), self.ourGoal)

    def midfielder_robotCode(self, obstacleConfs, goaliePosition):
        """inner while loop for each robots"""
        # pass randomly now
        # TODO: pass where less people are


        # random place along field width, in zone 0
        random_x_mag = np.random.rand()*0.455/2
        random_x_dir = np.sign(np.random.randn())

        # random place along field length, in zone 0
        random_y = np.random.rand()*0.4

        self.passPos = [random_x_mag*random_x_dir, random_y]

        if not self.executing:
            self.p0 = self.ballEngine.getBallPose()
            self.path, self.status = passPath(
                self.getRobotConf(self.bot), self.p0, self.passPos, kick=False, vmax=15, vr=15, vKick=25)

            self.prunePath()
            # avoid any obstacles
            # if self.status != 2:
            self.multiObstacleAwarePath(obstacleConfs, 0.04)

            # avoid wall boundaries
            self.prunePath()

            self.executing = True

        self.followPath(self.getRobotConf(self.bot), self.status, rb=0.05)

    def attacker_robotCode(self, obstacleConfs, goaliePosition):
        """inner while loop for Attacker robot"""
        if not self.executing:
            self.p0 = self.ballEngine.getBallPose()
            self.target = aim(goaliePosition,self.color)
            self.path, self.status = passPath(self.getRobotConf(self.bot), self.p0, self.target, kick=True, vmax=15, vr=15, vKick=25)

            # self.path[2,:] *= (0.75 - np.random.randn()*0.25) # varied velocity

            self.prunePath()
            # if self.status != 2:
            self.multiObstacleAwarePath(obstacleConfs, 0.04)
            self.prunePath()

            self.executing = True

        self.followPath(self.getRobotConf(self.bot), self.status, rb=0.05)

    def dumb_robotCode(self, *args, **kwargs):
        """inner while loop for Dumb robot"""
        vRobot = v2PosB(self.getRobotConf(self.bot), np.array(self.ballEngine.getBallPose()) + 0.1*(np.random.rand(2)-0.5),20)
        self.setMotorVelocities(vRobot[0], vRobot[1])

class Dumb(base_robot.BaseRobotRunner):
    def __init__(self, *args, **kwargs):
        """init for Dumb robot"""
        super(Dumb, self).__init__(*args, **kwargs)

class DumbMaster(base_robot.MultiRobotCyclicExecutor):
    def __init__(self, *args, **kwargs):
        super(DumbMaster, self).__init__(*args, **kwargs)

    def run(self):

        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)

            t0 = time.time()
            while time.time() - t0 < 30:
                for bot in self.bots:
                    bot.robotCode()

class Master(base_robot.MultiRobotCyclicExecutor):
    def __init__(self, *args, **kwargs):
        super(Master, self).__init__(*args, **kwargs)
        self.color = None
        self.idash = IDash(framerate=0.005)

    def getClosestZone(self, pose):
        """ Zones will be from downfield (0) to close to own goal (1) """
        pose = np.array(pose[:2]) # x-y only
        assert pose.size == 2 # pose should be 2-D

        # 2 ZONES: 0 and 1
        two_zone_boundary_y = 0.01
        zone = pose[1] > two_zone_boundary_y
        if self.color == 'Red':
            zone = not zone
        return zone

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)

            # robots have been added
            self.color = self.bots[0].color
            self.originalRoles = ['attacker', 'midfielder', 'goalie']
            self.roles = ['attacker', 'midfielder', 'goalie']
            goalie = self.bots[2]

            activeidx = 0 # striker starts first

            t0 = time.time()
            while time.time() - t0 < 600:
                self.ballEngine.update()

                assert activeidx != 2 # now only have active 0,1
                activebot = self.bots[activeidx]

                offense = self.originalRoles[activeidx] == 'attacker'
                if offense:
                    if self.roles[1] != 'dumb':
                        self.roles[1] = 'attacker'
                else:
                    if self.roles[1] != 'dumb':
                        self.roles[1] = 'midfielder'

                secondaryidx = not activeidx

                ballx, bally = self.ballEngine.getBallPose()
                robotX = np.zeros(2)
                robotY = np.zeros(2)
                robotX[0], robotY[0], _ = self.bots[0].getRobotConf()
                robotX[1], robotY[1], _ = self.bots[1].getRobotConf()
                distance2ball = 9999
                for i in xrange(len(self.bots[:-1])):
                    dist = np.sqrt((ballx - robotX[i])**2 + (bally- robotY[i])**2)
                    if dist < distance2ball:
                        distance2ball = dist
                        closestRobot = i
                ballPosX, ballPosY = self.ballEngine.getBallPose()
                for idx in range(len(self.bots[:-1])):
                    if offense:
                        oppGoalieConf, oppGoalieIdx = self.findOppGoalieConf(return_index=True)
                        oppGoalieIdx = self.convertOppBotsIdx2BotsIdx(oppGoalieIdx)
                        if idx == closestRobot:
                            self.bots[idx].robotCode(
                                role=self.roles[idx],
                                obstacleConfs=self.getObstacleConfs([idx, oppGoalieIdx]), # dont ignore yourself, or the oppGoalie
                                goaliePosition = self.findOppGoalieConf())
                        else: # robot is not closest robot
                            if True: # be a bitch
                                passivePos = oppGoalieConf
                            else:
                                if ballPosX >= 0:
                                    passivePos = [-0.2, ballPosY, 0]
                                else: # ballPosX < 0
                                    passivePos = [0.2, ballPosY, 0]
                            self.bots[idx].secondaryCode(
                                    role=self.roles[idx],
                                    obstacleConfs=self.getObstacleConfs([idx]), # attack the goalie
                                    passivePos=passivePos)
                    # defense
                    else:
                        # secondary defender (original attacker being passive)
                        if idx == secondaryidx:
                            if self.bots[idx].color == 'Red':
                                passivePos = [ballPosX, 0.2, 0]
                            else:
                                passivePos = [ballPosX, -0.2, 0]
                            self.bots[idx].secondaryCode(
                                role=self.roles[idx],
                                obstacleConfs=self.getObstacleConfs(secondaryidx),
                                passivePos=passivePos)
                        # primary defender (original defender being active)
                        else:
                            self.bots[idx].robotCode(
                                role=self.roles[idx],
                                obstacleConfs=self.getObstacleConfs(activeidx),
                                goaliePosition = self.findOppGoalieConf())

                goalie.robotCode(
                    role=self.roles[2],
                    obstacleConfs=self.getObstacleConfs(activeidx),
                    goaliePosition = self.findOppGoalieConf())

                def vizBots():
                    actx, acty = activebot.getRobotConf()[:2]
                    ballx, bally = self.ballEngine.getBallPose()
                    plt.hold('on')
                    try:
                        plt.plot(-self.activebot.passPos[1], self.activebot.passPos, 'ro')
                    except:
                        pass
                    plt.plot(-acty, actx, 'g+')
                    try:
                        plt.plot(-activebot.target[1], activebot.target[0], 'm*')
                    except:
                        pass
                    plt.plot(-oppGoalieConf[1], oppGoalieConf[0], 'yo')
                    plt.plot(-bally,ballx, 'ro')
                    plt.plot(-self.bots[secondaryidx].path[1,:], self.bots[secondaryidx].path[0,:], 'b.')
                    plt.plot(-activebot.path[1,:], activebot.path[0,:], 'g.')
                    plt.xlim([-0.8, 0.8]) # y axis in the field
                    plt.ylim([-0.7, 0.7]) # x axis in the field
                    plt.title('Red = RCV, Green = Active')
                    plt.xlabel('active path length: {}'.format(activebot.path.shape[1]))
                self.idash.add(vizBots)

                # if time.time() - activebot.time_started_2b_dumb > 2:
                #     self.roles = self.originalRoles[:]

                # if activebot.path.shape[1] < 5:
                #     self.roles[activeidx] = 'dumb'
                #     activebot.time_started_2b_dumb = time.time()

                p0 = activebot.p0
                p1 = self.ballEngine.getBallPose()
                p3 = self.ballEngine.getNextRestPos()
                dist_from_start = np.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)
                velocity_measure = np.sqrt((p1[0] - p3[0])**2 + (p1[1] - p3[1])**2)
                closest_zone = self.getClosestZone(p1)
                if dist_from_start > 0.01: # the ball has been touched
                #     if velocity_measure < 0.003: # wait til velocity reaches zero
                # if True:
                    # if velocity_measure < 1.0: # start kicking while ball is moving...
                    for bot in self.bots:
                        bot.executing = False
                    if closest_zone != activeidx: # success
                        # increment what is the new active zone
                        # self.setMotorVelocities(0, 0)
                        activeidx = not activeidx

                # self.idash.plotframe()

if __name__ == '__main__':
    import sys
    # cmd: python matchplay.py Blue
    # argv: ['matchplay.py', 'Blue']
    if len(sys.argv) < 2:
        color = 'Blue' # default
        oppColor = 'Red'
    else:
        color = sys.argv[1]
    if color == 'Blue':
        port=19998
        oppColor = 'Red'
    else:
        port=19999
        oppColor = 'Blue'

    # bluemaster = Master(ip='172.23.201.40', port=port)
    bluemaster = Master(port=port)
    # Order of which we addRobots kinda important...
    # self.bots -> Attacker, Midfielder, Goalie
    bluemaster.addRobot(Player(color=color, number=1, clientID=bluemaster.clientID))
    bluemaster.addRobot(Player(color=color, number=2, clientID=bluemaster.clientID))
    bluemaster.addRobot(Player(color=color, number=3, clientID=bluemaster.clientID))
    # MUST addOppRobots after, the list order depends on it...
    bluemaster.addOppRobot(Player(color=oppColor, number=1, clientID=bluemaster.clientID))
    bluemaster.addOppRobot(Player(color=oppColor, number=2, clientID=bluemaster.clientID))
    bluemaster.addOppRobot(Player(color=oppColor, number=3, clientID=bluemaster.clientID))

    bluemaster.run()