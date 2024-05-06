# -*- coding: utf-8 -*-
"""
Created on Fri May  3 14:44:13 2024
Path Creation using IFDS and Path Following using CCA3D
Note in this version:
    - No Local-Global adaptation 
    - Obstacles are not plotted
@author: Komsun
"""
import numpy as np
import matplotlib.pyplot as plt
from DynamicAutorouting import IFDS, FollowPath

def main():
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
    # Pre-allocation waypoints and paths
    if param['sim']['simMode'] == 1:
        # Since total nol of waypoints is fixed in this mode
        wp = np.zeros((3, param['sim']['tsim'] + 1))
        wp[:,0:1] = np.array([[x_init], [y_init], [z_init]])
    else:
        wp = np.array([[x_init], [y_init], [z_init]])
    
    # Path Following
    dtcum = 0
    v = param['uav']['speed']
    allTraj = np.array([[x_i], [y_i], [z_i]])
    for rt in range(param['sim']['rtsim']): 
        path = IFDS(param, wp)
        traj = FollowPath(path, param['uav']['state'], v)
        param['uav']['state'] = traj[:,-1]
        allTraj = np.append(allTraj, traj[0:3,:], axis=1)
    return (path, allTraj, param)

path, allTraj, param = main()
    
#%% Plot

ax = plt.figure().add_subplot(projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# Plot the trajectory
ax.plot(path[0, :], path[1, :], path[2, :], label='Path')
ax.plot(allTraj[0, :], allTraj[1, :], allTraj[2, :] ,label='Trajectory')
ax.legend()
ax.set_aspect('equal', adjustable='box')
plt.ion
plt.rcParams.update({'font.size': 20})
# plt.axis([0, 200, -100, 100, 0, 100])
# ax.set_zlim(0, 100)
# ax.set_xlim(0, 200)
# ax.set_ylim(-100, 100)
plt.show()


