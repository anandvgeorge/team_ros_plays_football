import vrep
import math
import time
import numpy as np
# https://docs.scipy.org/doc/numpy-dev/user/numpy-for-matlab-users.html
import matplotlib.pyplot as plt

def prox_sens_initialize(clientID, botName):
    """ Initialize proximity sensor. Maybe helpful later """
    proxSens=[]
    for i in range(4):
        handleName = '%s_proxSensor%d' % (botName, i+1)
        _, oneProxSens = vrep.simxGetObjectHandle(clientID, handleName, vrep.simx_opmode_oneshot_wait)
        # call the prox sensor once to start it
        _ = vrep.simxReadProximitySensor(clientID, oneProxSens, vrep.simx_opmode_streaming)
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
    for i in range(4):
        proxOut=vrep.simxReadProximitySensor(clientID, proxSens[i], vrep.simx_opmode_buffer)
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

def v2Pos(robotConf, finalPos, v = 20, k=3.5, rb=0.03):
    """ return a velocity vector to achieve the
        given desired final position: finalPos[0]=x, finalPos[1]=y
        robotConf[0]=x, robotConf[1]=y, robotConf[2]=theta
        v is the absolute velocity of the robot and
        k is the rotation gain """
    x = finalPos[0] - robotConf[0]
    y = finalPos[1] - robotConf[1]
    norm = (x**2 + y**2)**.5    # euclidian norm
    if norm < rb:
        vx = vy = 0
    else:
        vx = x * v / norm    # velocity normalization
        vy = y * v / norm
    # transformation to robot frame
    cos = math.cos(robotConf[2]) # robot orientation unique vector
    sin = math.sin(robotConf[2])
    rvt = -cos*vx-sin*vy   # robot translational velocity ~~> ohmega
    rvf = -sin*vx+cos*vy   # robot forward velocity
    return [rvf, k*rvt]   # robot velocity (forward, transaltional)

def v2PosB(robotConf, finalPos, v = 20, k=2, rb=0.03):
    """v2pos with Backward implement"""
    V=v2Pos(robotConf, finalPos, v, k, rb)
    if V[0]<0:
        V[1]=-V[1]
    return V

def v2PosP(robotConf, finalPos, vmax=20, k=2, kp=400):
    """v2pos with P controller """
    x = finalPos[0] - robotConf[0]
    y = finalPos[1] - robotConf[1]
    norm = (x**2 + y**2)**.5    # euclidian norm
    v=kp*norm
    if v>vmax:
        v=vmax
    return v2PosB(robotConf, finalPos, v, k, rb=0.01)

def v2orientation(robotConf, finalConf, v = 20, k=3.5, rb=0.05, kr=15):
    """ return a velocity vector to achieve the
        given desired final position: finalPos[0]=x, finalPos[1]=y
        robotConf[0]=x, robotConf[1]=y, robotConf[2]=theta
        v is the absolute velocity of the robot and
        k is the rotation gain
        rb radius of the buffer zone
        kr is the proportionnal coeff for the rotation"""
    tol = 0.001
    if ((robotConf[0]-finalConf[0])**2+(robotConf[1]-finalConf[1])**2)**0.5>rb:
        return v2Pos(robotConf, finalConf, v, k)

    # -- old computation of angular difference
    # theta = finalConf[2]-robotConf[2]-math.pi/2
    # if math.fabs(theta)>math.pi:
    #     theta=2*math.pi-theta

    # -- non buggy computation of angular difference
    theta = ThetaRange.angleDiff(finalConf[2]-math.pi/2, robotConf[2])

    if math.fabs(theta)<tol:
        return (0,0)
    return (0, kr*k*theta)   # robot velocity (forward, transaltional)

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
    if np.linalg.norm(s-c)<r+tol:   # robot inside the circle, no solution for tangeant
        t = g-np.array([[1.5*q*cos],        # last point on the circle
                        [1.5*q*sin]])
        path = np.concatenate((t, g1), axis=1)
        status=2
        return path, status
    path = np.concatenate((t, g1), axis=1)
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

def passPath(robotConf, ballPos, finalBallPos, vmax=25, vr=15, r=0.08, kq=0.002, k=0.036, q_bias=0.04, hold=False, kick=False, vKick=30):
    """
    compute path and velocity for each node
    vmax is max velocity we want the robot to achieve for the point of motion
    vr is the velocity of the robot in the circle,
    r is the radius of the circle,
    kq is the coeaff to create the distance q from the ballPos to the circle tangeant point,
    k is the coefficient for the ball model: d(t) = vrobot*k*(1-exp(-a*t))
    q_bias is the additive term to create the distance q from the ballPos to the circle tangent point

    """
    d = ((finalBallPos[0]-ballPos[0])**2+(finalBallPos[1]-ballPos[1])**2)**0.5
    if kick:
        vf=vKick
    else:
        vf = d/k
    q = q_bias+kq*vf
    theta = math.atan2(finalBallPos[1]-ballPos[1], finalBallPos[0]-ballPos[0])   # atan2(y, x)
    finalConf = (ballPos[0], ballPos[1], theta)
    path, status = smoothPath(robotConf, finalConf, r, q)

    v = vr*np.ones((1, np.size(path,1)))
    path = np.concatenate((path, v), axis=0)
    path[-1, -1]=vf

    # if we should hold onto the ball while kicking (inspired by other group)
    if hold:
        g = np.array([[finalConf[0]],   # goal
                      [finalConf[1]]])
        cos = math.cos(finalConf[2])
        sin = math.sin(finalConf[2])
        after_goal = []
        before_goal = []
        rb=0.025 # radius of ball
        for i in range(1, 5):
            after_goal.append(g+np.array([[i*rb*cos],       # points after the goal to be sure to reach it
                                          [i*rb*sin]]))
            before_goal.append(g-np.array([[i*rb*cos],      # points before the goal to be sure to be aligned
                                           [i*rb*sin]]))
        linePathAfter = np.column_stack(after_goal)
        linePathBefore = np.column_stack(before_goal)
        linePathVel = vf*np.ones((1, np.size(linePathAfter,1)))
        linePathAfter = np.concatenate((linePathAfter, linePathVel), axis=0)
        linePathBefore = np.concatenate((linePathBefore, linePathVel), axis=0)
        # add a line between the ball and the finalBallPos to the path
        path = np.column_stack((path[:,:-1], linePathBefore, path[:,-1], linePathAfter))
    if (status==0):
        path[-1, 0]=vmax
        path[-1, 1]=vmax
    return path, status

def calculatePathTime(path, kv=66.65):     #kv=66.65=vmot/vrobot
    """ return the estimated time for the robot to follow this path """
    t=0
    for i in range(1, np.size(path,1)):
        d = np.linalg.norm(path[0:2, i]-path[0:2, i-1])
        t =t + d/path[2, i]
    return t*kv

def interpolatePath(path,robotPosition):
    path = path[:,1:]
    pathNew = np.zeros((3,1))
    pathNew[0,0] = robotPosition[0]
    pathNew[1,0] = robotPosition[1]
    pathNew[2,0] = path[2,0]
    dist = math.sqrt((pathNew[0,0]-path[0,0])**2+(pathNew[1,0]-path[1,0])**2)
    interpolFactor = 3*int(math.ceil(dist*50)) # TODO
    for i in range(1,interpolFactor-1):
        temp = i*(path[0:2,0]-pathNew[0:2,0])/float(interpolFactor) + pathNew[0:2,0]
        pathNew = np.column_stack((pathNew,np.array([temp[0],temp[1],path[2,0]]).reshape((3,1))))
    pathNew = np.column_stack((pathNew,path))
    return pathNew

def obstacleDetector(obstacleConf, path,rb=0.025):
    #rb = 0.025 # TODO
    #obstacleNo = 0
    #for obstacleConf in obstacleConfs:
    #index = np.zeros(len(obstacleConfs),1)
    index = []
    distance = []
    # print len(path[0,:])
    for i in range(len(path[0,:])):
        dis = math.sqrt((obstacleConf[0]-path[0,i])**2 + (obstacleConf[1]-path[1,i])**2)
        # print dis
        if dis <= 2*rb and dis != 0: # TODO
            # print "WARNING: Obstacle detected"
            index.append(i)
            distance.append(dis)
        #obstacleNo += 1
    return (index, distance)

def avoidObstacle(path,obstacleConf,index,distance,rb=0.025):
    #rb = 0.025 #TODO
    minimalDistance = 2*rb #TODO
    for i in range(len(index)):
        delta = 0
        if (path[0,index[i]] == obstacleConf[0]) and (path[1,index[i]] == obstacleConf[1]):
            print 'TODO'
        c = math.sqrt((obstacleConf[0]+minimalDistance - path[0,index[i]])**2 + (obstacleConf[1] - path[1,index[i]])**2)
        gamma = math.acos((minimalDistance**2 + distance[i]**2 - c**2)/(2*minimalDistance*distance[i]))
        if path[1,index[i]] < obstacleConf[1]:
            gamma = -gamma
        #deltaX, deltaY = ThetaRange.pol2cart(minimalDistance,gamma)
        deltaX = minimalDistance * math.cos(gamma)
        deltaY = minimalDistance * math.sin(gamma)
        path[0,index[i]] = deltaX + obstacleConf[0]
        path[1,index[i]] = deltaY + obstacleConf[1]
    return path

def test_avoidObstacle():
    vector = np.zeros((3,2))
    vector[:,1] = [-0.5,0.5,0]
    robotPosition = (1,1)
    path = interpolatePath(vector,robotPosition)
    pathOld = path.copy()
    print path
    obstacleConf = (0.45,0.4)
    index, distance = obstacleDetector(obstacleConf,path)
    pathNew =  avoidObstacle(path,obstacleConf,index,distance)
    phi = np.linspace(0, 2*np.pi, 18, endpoint=True)
    x,y = 2*0.025*np.cos(phi)+obstacleConf[0], 2*0.025*np.sin(phi)+obstacleConf[1]

    actx, acty = vector[:2]
    ballx, bally = robotPosition[:2]
    print pathOld
    plt.hold('on')
    plt.plot(x,y,'y.')
    plt.plot(pathOld[0,:], pathOld[1,:], 'g.')
    plt.plot(pathNew[0,:], pathNew[1,:], 'b*')
    plt.plot(obstacleConf[0],obstacleConf[1])
    plt.plot(bally, ballx, 'b.')
    plt.plot(acty, actx, 'g+')
    plt.xlim([-1, 1]) # y axis in the field
    plt.ylim([-1, 1]) # x axis in the field
    plt.show()
    time.sleep(10)

#test_avoidObstacle()

def force_repulsion(k_repulse, rho, rho_0):
    """
    k_repulse: positive scaling factor
    rho: distance from point to obstacle
    rho_0: distance of influence
    """
    if rho <= rho_0:
        return k_repulse*(1.0/rho - 1.0/rho_0)*(1.0/rho**2)
    else:
        return 0

def aim(goaliePosition, ownColor):
    ballRadius = 0.05
    leftGoalPost = [0.2, 0.75] # position of left goal post
    rightGoalPost = [-0.2, 0.75]# position of right goal post
    tolerance = 0.01
    sign = 1
    gapRight = abs(goaliePosition[0] - rightGoalPost[0])
    gapLeft = abs(goaliePosition[0] - leftGoalPost[0])

    if gapRight >= gapLeft:
        xAim = rightGoalPost[0] + ballRadius + tolerance
    else:
        xAim = leftGoalPost[0] - ballRadius - tolerance

    if ownColor == 'Blue':
        sign = -1
    return [xAim, 0.75*sign]

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

