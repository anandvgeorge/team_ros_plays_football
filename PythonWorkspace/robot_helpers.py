import vrep

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

def vomega2bytecodes(v, omega, g, L=0.216):
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