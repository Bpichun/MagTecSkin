#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
"""
Created on Mon May 12 17:14:21 2025

@author: benjamin
"""

import numpy as np 


# ---- Sensor physical parameters ----
DiskRadius = 2
BoxROITolerance = 0.2
MagnetBoxTolerance = 0.1

SensorHeight = 3
SensorWidth = 20
SensorLength = 30

PoissonRatio = 0.4
YoungsModulus = 800000

ArticulationAngle = -0   # degrees
ArticulationAngle = np.deg2rad(ArticulationAngle)   

DeltaPositionSensor = 1
mu_mag_delGráfico =   4.627195188680999e-08


# ---- Generate Grid ----
GridMargin = 10 
Gridrows = 1
Gridcols = 2
NMagnets = Gridcols*Gridrows
MagnetSide = 1
CutMargin = GridMargin*0.2


# ---- Generate grid points (plane XY) ----
x = np.linspace(-(SensorLength - GridMargin)/2, (SensorLength - GridMargin)/2, Gridcols)
y = np.linspace(-(SensorWidth - GridMargin)/2, (SensorWidth- GridMargin)/2, Gridrows)
X, Y = np.meshgrid(x, y)

# ---- 2D grid points ----
GridPoints = np.column_stack((X.ravel(), Y.ravel()))


# ---- Magnets centers 3D coordinates ---- 
MagnetCenters = []
MagnetCenters.append([-7.5, 0, 0])
MagnetCenters += [[px, py, SensorHeight / 2] for px, py in GridPoints]
MagnetFreeCenters = MagnetCenters[1:]
# print(MagnetCenters)


# ---- Sensors centers 3D coordinates ---- 
SensorCenters = []
SensorCenters += [[px, py, -SensorHeight/2] for px, py in GridPoints]

# ---- Index for Sensor Centers ---- 
indexPerPointSensor = []
for center in SensorCenters:
    if center[0] > MagnetBoxTolerance:
        index = 0
    else:
        index = 1
    indexPerPointSensor.append(index)
            
        

# ---- BoxROI fixed  ----
BoxROIFixCoords = [
                    SensorLength/2 + BoxROITolerance,
                    SensorWidth/2 + BoxROITolerance, 
                    BoxROITolerance, 
                    -(-CutMargin + BoxROITolerance), 
                    -(SensorWidth/2 + BoxROITolerance), 
                    -BoxROITolerance
                  ]


# ---- BoxROI rigidified ----
BoxROIFixCoords1 = [
                    -(SensorLength / 2 + BoxROITolerance),   
                    -(SensorWidth / 2 + BoxROITolerance),   
                    -SensorHeight / 2 - BoxROITolerance,     
                    0,                                     
                    (SensorWidth / 2 + BoxROITolerance),    
                    BoxROITolerance      
                ]


# ---- BoxROI coordinates for magnets ----
MagnetBoxCoords = []

# Add fixed rigidified region 
MagnetBoxCoords.append(BoxROIFixCoords1)

# Add BoxROI for each magnet
for point in GridPoints:
    px, py = point
    pz = SensorHeight / 2  
    box = [
        px - (MagnetSide / 2 + MagnetBoxTolerance),  
        py - (MagnetSide / 2 + MagnetBoxTolerance),  
        pz - (MagnetSide / 2 + MagnetBoxTolerance),  
        px + (MagnetSide / 2 + MagnetBoxTolerance),  
        py + (MagnetSide / 2 + MagnetBoxTolerance),  
        pz + (MagnetSide / 2 + MagnetBoxTolerance)   
    ]
    MagnetBoxCoords.append(box)


# --- IndexPairs for SubsetMultiMapping ---
IndexPairs = [0, 1]  # Fixed region mapping
for i in range(NMagnets):
    IndexPairs.extend([1, i])  # Map each magnet
    
    
