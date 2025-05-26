#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
"""
Created on Mon May 12 17:14:21 2025

@author: benjamin
"""
import numpy as np 

BoxROITolerance = 0.2
MagnetBoxTolerance = 0.1
SensorHeight = 3
SensorWidth = 20
SensorLength =50

PoissonRatio = 0.4
YoungsModulus = 2000

#Generate grid
GridMargin = 10 
Gridrows = 3
Gridcols = 5
NMagnets = Gridcols*Gridrows
MagnetSide = 1
# MagnetPosition = [0, 0, SensorHeight/2]

DeltaPositionSensor = 1
mu_mag_delGráfico =   4.627195188680999e-08

x = np.linspace(-(SensorLength - GridMargin)/2, (SensorLength - GridMargin)/2, Gridcols)
y = np.linspace(-(SensorWidth - GridMargin)/2, (SensorWidth- GridMargin)/2, Gridrows)
X, Y = np.meshgrid(x, y)
GridPoints = np.column_stack((X.ravel(), Y.ravel()))
MagnetCenters = [[px, py, SensorHeight / 2] for px, py in GridPoints]


# print(GridPoints)
BoxROIFixCoords = [SensorLength/2 + BoxROITolerance,
                   SensorWidth/2 + BoxROITolerance, 
                   BoxROITolerance, 
                   -(SensorLength/2 + BoxROITolerance), 
                   -(SensorWidth/2 + BoxROITolerance), 
                   -BoxROITolerance ]

# MagnetBoxCoords = [MagnetPosition[0] - (MagnetSide/2 + MagnetBoxTolerance), 
#                     MagnetPosition[1] - (MagnetSide/2 + MagnetBoxTolerance), 
#                     MagnetPosition[2] - (MagnetSide/2 + MagnetBoxTolerance), 
#                     MagnetPosition[0] + MagnetSide/2 + MagnetBoxTolerance, 
#                     MagnetPosition[1] + MagnetSide/2 + MagnetBoxTolerance, 
#                     MagnetPosition[2] + MagnetSide/2 + MagnetBoxTolerance ]



MagnetBoxCoords = []

for point in GridPoints:
    px, py = point
    pz = SensorHeight / 2
    box = [
        px - (MagnetSide/2 + MagnetBoxTolerance),
        py - (MagnetSide/2 + MagnetBoxTolerance),
        pz - (MagnetSide/2 + MagnetBoxTolerance),
        px + (MagnetSide/2 + MagnetBoxTolerance),
        py + (MagnetSide/2 + MagnetBoxTolerance),
        pz + (MagnetSide/2 + MagnetBoxTolerance)
    ]
    MagnetBoxCoords.append(box)
   