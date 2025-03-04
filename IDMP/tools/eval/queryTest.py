'''
    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
    Copyright (C) 2024 Usama Ali

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License v3 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License v3 for more details.

    You should have received a copy of the GNU General Public License v3
    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.

    Authors: Usama Ali <usama.ali@thws.de>
             Adrian Mueller <adrian.mueller@thws.de>
             Lan Wu <Lan.Wu-2@uts.edu.au>
'''

from idmp_ros.srv import GetDistanceGradient
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import os
import rospy
import sys
import time

matplotlib.use("Qt5Agg")

def getVoxelCenters(voxSize, xMin, xMax, yMin, yMax, zMin, zMax):
    xMinBound = round(int(xMin/voxSize) * voxSize + voxSize/2,4) #norm range to voxels and then shift by halflenght
    xMaxBound = round(int(xMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4) 
    yMinBound = round(int(yMin/voxSize) * voxSize + voxSize/2,4)
    yMaxBound = round(int(yMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    zMinBound = round(int(zMin/voxSize) * voxSize + voxSize/2,4)
    zMaxBound = round(int(zMax/voxSize) * voxSize + voxSize/2 - voxSize/4,4)
    return np.mgrid[xMinBound:xMaxBound:voxSize, yMinBound:yMaxBound:voxSize, zMinBound:zMaxBound:voxSize]

if __name__=="__main__":

    # rospy.init_node("evalTest")
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
    
    x = []
    t = []
    grids = []
    for i in np.arange(0.05, 1, 0.05):
        tmp = getVoxelCenters(i, -2, 2, -2, 0.8 , 0.2 , 1.5)
        if not tmp.size/3 in x:
             grids.append(tmp.reshape(3,-1).T)
             x.append(tmp.size/3)
    
    plt.rcParams['figure.constrained_layout.use'] = True

    for g in grids:             
        #flatten grid to send to service 
        ser_grid = np.reshape(g, len(g)*3)
        #query idmp
        startT = time.time_ns()
        res = query(ser_grid)
        t.append((time.time_ns()-startT)/1e3)

    print("Timing:")
    print(x)
    print(t)
    plt.plot(x,t)
    plt.show()
    dt = []
    for i in range(len(x)):
        dt.append(t[i]/x[i])
    print(dt)
    plt.plot(x,dt)
    plt.show()
            
