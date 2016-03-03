import vrep
import math
import numpy as np
import matplotlib.pyplot as plt

def prox_sens_initialize(clientID):
    """ Initialize proximity sensor. Maybe helpful later """
    proxSens=[]
    for i in range(8):
        _, oneProxSens = vrep.simxGetObjectHandle(clientID, 'ePuck_proxSensor%d' % (i+1), vrep.simx_opmode_streaming)
        proxSens.append(oneProxSens)
    return proxSens

def prox_sens_read(clientID, proxSens):
    """ Read the proximity sensor
    clientID: vrep clientID
    proxSens: array of proxSensor handles
    """
    outputs = []
    keys = ('returnCode','detectionState','detectedPoint','detectedObjectHandle','detectedSurfaceNormalVector')
    # NOTE: take norm of deteected point to get the distance
    for i in range(8):
        proxOut=vrep.simxReadProximitySensor(clientID, proxSens[i], vrep.simx_opmode_streaming)
        outputs.append(dict(zip(keys, proxOut)))
    return outputs

def vomega2bytecodes(v, omega, g, L=0.260):
    """
    Turns v and omega into control codes for differential drive robot

    Parameters
    ----------
    v: forward velocity
    omega: angular velocity
    g: gain constant
    L: length of axle, default .216
        >>> right = -1.9914
        >>> left = -2.2074
        >>> right - left
        0.21599999999999975

    Returns
    -------
    ctrl_sig_left: number to give to simxSetJointTargetVelocity
    ctrl_sig_right: number to give to simxSetJointTargetVelocity
    """
    v_comm = v
    v_diff = omega * L / 2.0
    ctrl_sig_left = (v_comm - v_diff) / float(g)
    ctrl_sig_right = (v_comm + v_diff) / float(g)
    return ctrl_sig_left, ctrl_sig_right

class ThetaRange(object):
    """ Class to organize methods related to shifts and transformations of Theta or Angles """

    @staticmethod
    def normalize_angle_pos(angle):
        return ((angle % (2*np.pi)) + 2*np.pi) % (2*np.pi)

    @staticmethod
    def normalize_angle(angle):
        """ Constrains the angles to the range [0, pi) U [-pi, 0) """
        a = ThetaRange.normalize_angle_pos(angle)
        if a >= np.pi:
            a -= 2*np.pi
        return a

    @staticmethod
    def angleDiff(a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
            the difference is always based on the closest rotation from angle a to angle b
            examples:
                angleDiff(.1,.2) -> -.1
                angleDiff(.1, 2*math.pi - .1) -> .2
                angleDiff(.1, .2+2*math.pi) -> -.1
        """
        a = ThetaRange.normalize_angle(a)
        b = ThetaRange.normalize_angle(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    @staticmethod
    def test_angleDiff():
        angles = np.linspace(-np.pi, np.pi, 64)
        theta1s = np.linspace(-np.pi, np.pi, 8)
        for count, theta1 in enumerate(theta1s):
            diffs = []
            for theta0 in angles:
                diffs.append(ThetaRange.angleDiff(theta0, theta1))
            plt.subplot(4,2,count)
            plt.plot(diffs)
        plt.pause(15)

    @staticmethod
    def cart2pol(x, y):
        """ NOTE: arctan2 returns phi in the range [-pi, pi] """
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return(rho, phi)

    @staticmethod
    def pol2cart(rho, phi):
        """ where rho is the Radius, and phi is the angle """
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)

    @staticmethod
    def test_pol2cart2pol():
        # pol2cart
        rho = 1
        angles = np.linspace(0, 2*np.pi, 64)
        xs, ys = zip(*[pol2cart(rho, angle) for angle in angles])

        plt.subplot(3,1,1)
        plt.plot(angles)

        # cart2pol
        new_rhos, new_angles = zip(*[cart2pol(x,y) for x, y in zip(xs, ys)])
        plt.subplot(3,1,2)
        plt.plot(new_angles)
        plt.subplot(3,1,3)
        plt.plot([ThetaRange.normalize_angle(ang) for ang in angles])
        plt.pause(15)

# ThetaRange.test_angleDiff()