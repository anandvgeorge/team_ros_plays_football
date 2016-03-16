import vrep
import math
import numpy as np
# https://docs.scipy.org/doc/numpy-dev/user/numpy-for-matlab-users.html
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

def v2Pos(robotConf, finalPos, v = 20, k=3.5):
    """ return a velocity vector to achieve the 
        given desired final position: finalPos[0]=x, finalPos[1]=y 
        robotConf[0]=x, robotConf[1]=y, robotConf[2]=theta 
        v is the absolute velocity of the robot and 
        k is the rotation gain """
    x = finalPos[0] - robotConf[0]
    y = finalPos[1] - robotConf[1]
    norm = (x**2 + y**2)**.5    # euclidian norm
    if norm == 0:
        vx = vy = 0
    else:       
        vx = x * v / norm    # velocity normalization
        vy = y * v / norm
    # transformation to robot frame
    cos = math.cos(robotConf[2]) # robot orientation unique vector
    sin = math.sin(robotConf[2])
    rvt = -cos*vx-sin*vy   # robot forward velocity
    rvf = -sin*vx+cos*vy   # robot translational velocity ~~> ohmega
    return (rvf, k*rvt)   # robot velocity (forward, transaltional)

def smoothPath(robotConf, finalConf, r=0.08, q=0.08, theta=math.pi/10, rb=0.025):
    """ inital configuration of the robot, final Configuration of the robot
        radius of arc for path and distance q for a staight line to the final pos
        theta is the angle to set the number of point on the circle arc
        see figure in lecture 4 locomotion p.41
        return the path and a status:
        0=complete path, 1=p0 is missing, 2=radius curve is missing
    """
    status=1
    tol = .0001
    cos = math.cos(finalConf[2]) 
    sin = math.sin(finalConf[2])  
    g = np.array([[finalConf[0]],   # goal
                  [finalConf[1]]])
    t = g-np.array([[q*cos],        # last point on the circle
                    [q*sin]])
    g1 = g+np.array([[rb*cos],       # point after the goal to be sure to reach it
                     [rb*sin]])
    c1 = t+np.array([[-r*sin],      # center of the circle1 
                     [r*cos]])
    c2 = t-np.array([[-r*sin],      # center of the circle2
                     [r*cos]]) 
    s = np.array([[robotConf[0]],       # start: robot position
                  [robotConf[1]]])  
    if (np.linalg.norm(s-c1) < np.linalg.norm(s-c2)):
        c = c1  # choose the circle the closest to the robot 
        gamma = -math.pi/2   # anticlockwise
    else:
        c = c2
        gamma = math.pi/2  # clockwise
    b1 = math.atan2(c[1,0]-s[1,0], c[0,0]-s[0,0])   # atan2(y, x)   
    path = np.concatenate((t, g1), axis=1)
    if np.linalg.norm(s-c)<r+tol:   # robot inside the circle, no solution for tangeant
        status=2
        return path, status
    d = np.linalg.norm(s-c)    
    sgnG = np.sign(gamma) 
    b2 = math.asin(r/d)*sgnG
    p = c+np.array([[r*np.cos(b1+b2+gamma)],    # first point on the circle
                  [r*np.sin(b1+b2+gamma)]])    
    n = np.linalg.norm(s-p)
    alpha1=math.atan2(p[1,0]-c[1,0], p[0,0]-c[0,0])     # atan2(y, x)   
    alpha2=math.atan2(t[1,0]-c[1,0], t[0,0]-c[0,0])     # atan2(y, x)   
    alpha=math.fabs(alpha1-alpha2)
    if (alpha2>alpha1 and gamma>0) or (alpha2<alpha1 and gamma<0):
        alpha = 2*math.pi-alpha
    print np.rad2deg(alpha)     
    if q<n: # if we have enough space to add a point between s and p    
        status=0                                    
        p0=p+q*(s-p)/n # add p0 at a distance q from p
        p = np.concatenate((p0, p), axis=1) 
    n = np.ceil((alpha)/theta) # nb of point on the circle      
    for i in np.arange(theta, n*theta, theta):  
        p = np.concatenate((p, c+np.array([[r*np.cos(b1+b2+gamma-i*sgnG)], # point on the circle
                                         [r*np.sin(b1+b2+gamma-i*sgnG)]])), axis=1)
    path = np.concatenate((s, p, path), axis=1)
    return path, status

def passPath(robotConf, ballPos, finalBallPos, vr=15, r=0.08, kq=0.002, k=0.036): 
    """
    compute path and velocity for each node
    vr is the velocity of the robot in the circle, 
    r is the radius of the circle,
    kq is the coeaff to create the distance q from the ballPos to the circle tangeant point,
    k is the coefficient for the ball model: d(t) = vrobot*k*(1-exp(-a*t))
    """
    vmax = 25
    d = ((finalBallPos[0]-ballPos[0])**2+(finalBallPos[1]-ballPos[1])**2)**0.5
    vf = d/k
    q = 0.04+kq*vf
    theta = math.atan2(finalBallPos[1]-ballPos[1], finalBallPos[0]-ballPos[0])   # atan2(y, x)   
    finalConf = (ballPos[0], ballPos[1], theta)  
    path, status = smoothPath(robotConf, finalConf, r, q)
    
    v = vr*np.ones((1, np.size(path,1)))
    path = np.concatenate((path, v), axis=0)
    path[-1, -1]=vf
    if (status==0):
        path[-1, 0]=vmax 
        path[-1, 1]=vmax    
    return path

def calculatePathTime(path, kv=66.65):     #kv=66.65=vmot/vrobot
    """ return the estimated time for the robot to follow this path """
    t=0
    for i in range(1, np.size(path,1)):
        d = np.linalg.norm(path[0:2, i]-path[0:2, i-1])
        t =t + d/path[2, i]   
    return t*kv
                    
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

    