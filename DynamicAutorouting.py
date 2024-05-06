# -*- coding: utf-8 -*-
"""
Created on Fri May  3 14:28:46 2024

@author: Komsun
"""
import numpy as np
import math

##############################################################################

def Setting(num=1):
    """
    Initialize neccessary parameters for the Dynamic Autorouting program

    Args:
        num (int, optional): Setting profile number. Defaults to 1.

    Returns:
        param (dict): Dictionary of parameters.
    """
    if num == 1:
        (x_init, y_init, z_init) = (0, 0, 0)
        (x_i, y_i, z_i) = (0, -10, 5)
        (psi_i, gamma_i) = (0, 0)
        (x_f, y_f, z_f) = (200, 0, 10)
        
        param = {
            'ifds': {
                'rho0': 2.5, 
                'sigma0': 0.01, 
                'sf': 0},
            'uav' : {
                'state': np.array([x_i, y_i, z_i, psi_i, gamma_i]),
                'start location': np.array([x_init, y_init, z_init]),
                'destin': np.array([x_f, y_f, z_f]),
                'speed': 10.0},
            'sim' : {
                'rt': 1, 
                'dt': 0.1, 
                'tsim': 50, 
                'rtsim': 80, 
                'scene': 2,
                'targetThresh': 10,
                'simMode':2}
            }
    elif num == 2: # For PyBullet testing
        (x_init, y_init, z_init) = (0, 0, 0)
        (x_i, y_i, z_i) = (0, -2, 0)
        (psi_i, gamma_i) = (0, 0)
        (x_f, y_f, z_f) = (10, 0, 1)
        
        param = {
            'ifds': {
                'rho0': 2.5, 
                'sigma0': 0.01, 
                'sf': 0},
            'uav' : {
                'state': np.array([x_i, y_i, z_i, psi_i, gamma_i]),
                'start location': np.array([x_init, y_init, z_init]),
                'destin': np.array([x_f, y_f, z_f]),
                'speed': 0.2},
            'sim' : {
                'rt': 1, 
                'dt': 0.1, 
                'tsim': 50, 
                'rtsim': 80, 
                'scene': 3,
                'targetThresh': 0.5,
                'simMode':2}
            }
    return param

##############################################################################

def Main(setting=1):
    """
    Main loop for the Dynamic Autorouting Program: Creating Path and Trajectory

    Args:
        setting (int, optional): Setting profile number. Default to 1.

    Returns:
        path (Array 3x_): A 3D path generated from IFDS algorithm.
        allTraj (Array 3x_): A 3D trajectory generated from CCA3D algorithm.
        param (dict): A dictionary of various parameters.

    """
    param = Setting(setting)
    # Pre-allocation waypoints and paths
    if param['sim']['simMode'] == 1:
        # Since total nol of waypoints is fixed in this mode
        wp = np.zeros((3, param['sim']['tsim'] + 1))
        wp[:,0:1] = param['uav']['start location'].reshape(3,1)
    else:
        wp = param['uav']['start location'].reshape(3,1)
    
    # Path Following
    dtcum = 0
    v = param['uav']['speed']
    allTraj = param['uav']['state'][:3].reshape(3,1)
    for rt in range(param['sim']['rtsim']): 
        path = IFDS(param, wp)
        traj = FollowPath(path, param['uav']['state'], v)
        param['uav']['state'] = traj[:,-1]
        allTraj = np.append(allTraj, traj[0:3,:], axis=1)
    return (path, allTraj, param)

##############################################################################

def CreateScene(num, loc, rt):
    """
    Create scenarios with obstacles

    Args:
        num (int): Scenario number.
        loc (Array 3x1): Current UAV's position.
        rt (int): Current time step.

    Returns:
        Obj (list): A list of dictionaries containing obstacles' informations.
    """
    
    # Initilize an empty list
    Obj = []
    
    (X, Y, Z) = loc
    def Shape(shape, x0, y0, z0, D, h=0):
        def CalcGamma():
            Gamma = ((X - x0)/a)**(2*p) + ((Y - y0)/b)**(2*q) + ((Z - z0)/c)**(2*r)
            return Gamma
        def CalcDg():
            dGdx = (2*p*((X - x0)/a)**(2*p - 1))/a
            dGdy = (2*q*((Y - y0)/b)**(2*q - 1))/b
            dGdz = (2*r*((Z - z0)/c)**(2*r - 1))/c
            return (dGdx, dGdy, dGdz)
        
        nonlocal numObj 
        numObj += 1
        if shape == "sphere":
            (a, b, c) = (D/2, D/2, D/2)
            (p, q, r) = (1, 1, 1)
        elif shape == "cylinder":
            (a, b, c) = (D/2, D/2, h)
            (p, q, r) = (1, 1, 4)
        elif shape == "cone":
            (a, b, c) = (D/2, D/2, h)
            (p, q, r) = (1, 1, 0.5)
        
        Gamma = CalcGamma()
        (dGdx, dGdy, dGdz) = CalcDg()
        n = np.array([[dGdx], [dGdy], [dGdz]])
        t = np.array([[dGdy],[-dGdx], [0]])
        origin = np.array([x0, y0, z0])
        # Add a new object to the list
        Obj.append({'Gamma': Gamma, 'n': n, 't': t, 'origin': origin})
        # Obj[numObj]['Gamma'] = Gamma 
        # Obj[numObj]['n'] = n
        # Obj[numObj]['t'] = t
    
    numObj = 0
    if   num == 1:
        Shape("sphere", 50, 5, 0, 50)
    elif num == 2:
        Shape("cylinder", 60, 5, 0, 50, 50)
        Shape("sphere", 120, -10, 0 ,50)
    elif num == 3:
        Shape("sphere", 5, 5, 0, 5)
        Shape("cylinder", 6, 0, 0, 5, 5)
        Shape("sphere", 12, -1, 0 ,5)
    
    return Obj

###############################################################################

def IFDS(param, wp):
    """
    Generate 3D path using Interfered Fluid Dynamical System (IFDS) algorithm

    Args:
        param (dict): Contain all essential parameters for UAV and simulation.
        wp (Array 3x1): Initial location to generate a path from.

    Returns:
        Path (Array 3x_): A 3D path.

    """
    def CalcUBar(param, loc, Obj):
        """
        Calculate modified velocity UBar for the IFDS algorithm

        Args:
            param (dict): Contain all essential parameters for UAV and simulation.
            loc (Array 3x1): Current location of the UAV (x, y, z).
            Obj (list): A list of dictionaries containing information of the obstacles.

        Returns:
            UBar (Array 3x1): The modified velocity vector.
        """
        # Load parameters
        destin = param['uav']['destin']
        C      = param['uav']['speed']
        rho0   = param['ifds']['rho0']
        sigma0 = param['ifds']['sigma0']
        (X, Y, Z)=  loc
        (xd, yd, zd) = destin
        
        dist = np.linalg.norm(loc - destin)
        u = -np.array([[C*(X - xd)/dist],
                       [C*(Y - yd)/dist],
                       [C*(Z - zd)/dist]])
        # Pre-allocation
        Mm = np.zeros((3,3))
        sum_w = 0;
        for j in range(len(Obj)):
            # Reading Gamma for each obstacle
            Gamma = Obj[j]['Gamma']
            # Unit normal and tangential vector
            n = Obj[j]['n']
            t = Obj[j]['t']
            dist_obs = np.linalg.norm(loc - Obj[j]['origin'])
            ntu = np.dot(np.transpose(n), u)
            if ntu < 0 or param['ifds']['sf'] == 1:
                rho   = rho0   * math.exp(1 - 1/(dist_obs * dist))
                sigma = sigma0 * math.exp(1 - 1/(dist_obs * dist))
                n_t = np.transpose(n)
                M = np.identity(3) - np.dot(n,n_t)/(abs(Gamma)**(1/rho)*np.dot(n_t,n)) + \
                    np.dot(t,n_t)/(abs(Gamma)**(1/sigma)*np.linalg.norm(t)*np.linalg.norm(n))
                
            elif ntu >= 0 and param['ifds']['sf'] == 0:
                M = np.identity(3)
            else:
                UBar = u
                return
    
            # Calculate Weight
            w = 1
            if len(Obj) > 1:
                w = [w*(Obj[i]['Gamma'] - 1)/((Obj[j]['Gamma'] - 1) + (Obj[i]['Gamma']-1)) for i in range(len(Obj)) if i!=j][0]
            # Saving into each obstacles
            Obj[j]["w"] = w
            Obj[j]["M"] = M
            sum_w = sum_w + w
        for j in range(len(Obj)):
            w_tilde = Obj[j]["w"]/sum_w
            Mm = Mm + w_tilde*Obj[j]["M"] 
        
        UBar = np.dot(Mm, u)
        return UBar
    
    def loop(wp, t):
        flagBreak = 0
        if t > 1000:
            flagBreak = 1 # break
        loc = wp[:, -1]
        Obstacle = CreateScene(scene, loc, rt)

        if np.linalg.norm(loc - destin) < targetThresh:
            print("Path found at step #" + str(t))
            wp = wp[:, :-1]
            # Path[rt] = wp
            foundPath = 1
            flagBreak = 1 # break
        else:
            UBar = CalcUBar(param, loc, Obstacle)
            if param['sim']['simMode'] == 1:
                # Since total no. of waypoints is fixed in this mode
                wp[:, t+1:t+2] = wp[:, t].reshape(3, 1)+ UBar * dt
            else:
                # Since total nol of waypoints are unknown in this mode
                wp = np.append(wp, wp[:, -1].reshape(3, 1)+ UBar * dt, axis=1)
            
            
        return (flagBreak, wp)  
        
    # ------------------Start of IFDS: Load Param data -------------------
    rt = param['sim']['rt']
    dt = param['sim']['dt']
    destin = param['uav']['destin']
    targetThresh = param['sim']['targetThresh']
    scene = param['sim']['scene']
    
    # Initialization
    Path = np.array([])
    foundPath = 0
    if param["sim"]["simMode"] == 1:
        # Mode 1: Simulate by limiting steps
        for t in range(param['sim']['tsim']):
            flagBreak, wp = loop(wp, t)
            if flagBreak:
                break
    else:
        # Mode 2: Simulate by reaching distance
        t = 0
        while True:
            flagBreak, wp = loop(wp, t)
            if flagBreak:
                break
            t += 1
        
    wp = wp[:, 0:t]
    Path = np.delete(wp, np.s_[t+1:len(wp)], axis=1)
    return Path        
        
##############################################################################

def CCA3D(Wi, Wf, uavStates, v):
    """
    Three-dimensional Carrot-Chasing Algorithm

    Args:
        Wi (Array 3x1): Initial waypoint.
        Wf (Array 3x1): Final waypoint.
        uavStates (Array 5x1): UAV's state (x, y, z, psi, gamma).
        v (float): UAV's cuising speed (m/s).

    Returns:
        updatedState (tuple): Updated UAV's state (x, y, z, psi, gamma).
    """
    
    def WrapAngle(angle):
        if angle < math.pi:
            angle = angle + 2*math.pi
        elif angle > math.pi:
            angle = angle - 2*math.pi
            
    def LimitAngle(angle):
        if angle > math.pi/2:
            angle = math.pi/2
        elif angle < -math.pi/2:
            angle = -math.pi/2
    
    def LimitAccel(u, umax):
        if u > umax:
            u = umax
        elif u < -umax:
            u = -umax
            
    Wi = np.array(Wi).reshape(3,1)
    Wf = np.array(Wf).reshape(3,1)
    
    dt = 0.01
    timeSpent = 0
    
    # Extract UAV states
    (x, y, z, psi, gamma) = uavStates
    
    # CCA Tuning Parameters
    kappa = 50
    delta = 20
    Rmin = 10           # UAV Minimum Turn Radius [m] (lower is better)
    umax = v**2 / Rmin  # UAV Maximum Lateral Acceleration
    
    i = 0  # Time index
    p = np.array([[x], [y], [z]]) # List of np arrays
    
    # Normal plane checker
    Rw_vect = Wf - Wi
    Rw = np.linalg.norm(Rw_vect)
    (ox, oy, oz) = Wf
    (a, b, c) = Rw_vect
    
    while a*(p[0] - ox) + b*(p[1] - oy) + c*(p[2] - oz) < 0:
        
        # Step 1: Distance between initial waypoing and current position, Ru
        Ru_vect = Wi - p
        Ru = np.linalg.norm(Ru_vect)
        # Step 2: Orientation of vector from initial waypoint to final waypoint, theta
        theta1 = math.atan2(Wf[1] - Wi[1], Wf[0] - Wi[0])
        theta2 = math.atan2(Wf[2] - Wi[2], math.sqrt( (Wf[0] - Wi[0])**2 + (Wf[1] - Wi[1])**2 ))
        # Step 3: Orientation of vector from initial waypoint to current UAV position, theta_u
        # theta_u1 = math.atan2(p[i][1] - Wi[1], p[i][0] - Wi[0])
        # theta_u2 = math.atan2(p[i][2] - Wi[2], math.sqrt( (p[i][0] - Wi[0])**2 + (p[i][1] - Wi[1])**2 ))
        # Step 4: Distance between initial waypoint and q, R
        if Ru != 0 and Rw != 0:
            alpha = math.atan( np.dot(np.transpose(Ru_vect), Rw_vect)/(Ru * Rw))
        else:
            alpha = 0
        R = math.sqrt(Ru**2 - (Ru*math.sin(alpha))**2)
        # Step 5: Carrot Posiiton, s = (xt, yt)
        xt = Wi[0] + (R + delta) * math.cos(theta2) * math.cos(theta1)
        yt = Wi[1] + (R + delta) * math.cos(theta2) * math.sin(theta1)
        zt = Wi[2] + (R + delta) * math.sin(theta2)
        # Step 6: Desired heading angle and pitch angle, psi_d gamma_d
        psi_d = math.atan2(yt - p[1], xt - p[0])
        gamma_d = math.atan2(zt - p[2], math.sqrt( (xt - p[0])**2 + (yt - p[1])**2))
        # Wrapping up angles
        WrapAngle(psi_d)
        WrapAngle(gamma_d)
        # Limiting angles
        LimitAngle(psi_d)
        LimitAngle(gamma_d)
        # Step 7: Guidance Yaw and Pitch Command, u1 u2
        del_psi = psi_d - psi
        del_gam = gamma_d - gamma
        u1 = kappa * del_psi * v
        u2 = kappa * del_gam * v
        # Limit acceleration
        LimitAccel(u1, umax)
        LimitAccel(u2, umax)
        
        # Final Step: Update UAV state
        dx = v * math.cos(gamma) * math.cos(psi)
        dy = v * math.cos(gamma) * math.sin(psi)
        dz = v * math.sin(gamma)
        dpsi = u1 / (v * math.cos(gamma))
        dgam = u2 / v
        
        x = x + dx * dt
        y = y + dy * dt
        z = z + dz * dt
        psi = psi + dpsi * dt
        gamma = gamma + dgam * dt
        
        timeSpent += dt
        p = np.array([[x], [y], [z]]) # List of np arrays
        i += 1
        if i > 10000:
            break
    updatedState = (x, y, z, psi, gamma, timeSpent)
    return updatedState

##############################################################################
        
def FollowPath(path, uavStates, v):
    """
    Three-dimensional path-following scheme

    Args:
        path (Array 3x_): A path to follow.
        uavStates (Array 5x1): Current UAV's states (x, y, z, psi, gamma).
        v (float): UAV's cuising speed (m/s).

    Returns:
        trajectory (Array 5x_): A 3D trajectory of the UAV.
    """
    
    # Extract UAV states
    x, y, z, psi, gamma = uavStates
    # Initialize Trajectory
    trajectory = np.array([[x], [y], [z], [psi], [gamma]])
    dtcum = 0
    for j in range(path.shape[1]-1):
        if dtcum >= 1:
            break
        Wi = path[:, j].reshape(3, 1)
        Wf = path[:, j+1].reshape(3, 1)
        
        path_vect = Wf - Wi
        a, b, c = path_vect
        # Check if the waypoint is ahead of current position
        if a*(x - Wf[0]) + b*(y - Wf[1]) + c*(z - Wf[2]) < 0:
            x, y, z, psi, gamma, timeSpent = CCA3D(Wi, Wf, uavStates, v)
            dtcum += timeSpent
            newState = np.array([[x], [y], [z], [psi], [gamma]])
            trajectory = np.append(trajectory, newState, axis=1)
    return trajectory
        