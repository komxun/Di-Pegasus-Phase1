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
from DynamicAutorouting import Main

path, allTraj, param = Main()
    

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


