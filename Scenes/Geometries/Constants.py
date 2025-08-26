#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
"""
Created on Mon May 12 17:14:21 2025

@author: benjamin
"""

"""
This script defines the geometry and physical parameters of a magnetic skin model with embedded magnets.
It specifies the positions of magnets and magnetic sensors, regions of interest for rigidification,
the articulation angle, and the mappings required for simulations in SOFA.
"""

import numpy as np 

'''
------------------------------------------------------------------------------------
                Here you can modify the simulation parameters 
------------------------------------------------------------------------------------
'''
# ---- Sensor physical parameters ----

DiskRadius = 2
DeltaPositionSensor = 1

# Additional margin to define the region of interest (Rigid).
BoxROITolerance = 0.2


# Extra tolerance around each magnet to define its interaction volume.
MagnetBoxTolerance = 0.1

# Physical dimensions of the MagneticSkin (in millimeters).
MagneticSkinHeight = 5      # Height (Z-axis)
MagneticSkinWidth = 20      # Width (Y-axis)
MagneticSkinLength = 50     # Length (X-axis)

# Mechanical properties of the sensor material (used for physical deformation simulations).
PoissonRatio = 0.4            # Poisson ratio of the material
YoungsModulus = 60000        # Youngs modulus (material stiffness)

# Magnetic moment magnitude
mu_magnitude = 4.627195188680999e-08     # Magnetic moment magnitude (in appropriate units)


#Small characteristic length factor = finer mesh
SurfaceMeshCharacteristicLength = 0.1    # finer mesh for surfaces (2D)
VolumeMeshCharacteristicLength = 0.25    # coarser mesh for volume (3D)


# ---- Indenter Parameters (SphereROI) ----
indenterRadius = 2                                      # Radius of the sphere used as the indenter
indenterPosition = [0, 0, MagneticSkinHeight]              # Initial position of the indenter (on the top surface)
indenterMotion = False                                  # Enables or disables motion of the indenter
identadorForce = [0, 0, -1000000]                           # Total force applied by the indenter (negative Z direction)

# Articulation parameters
ArticulationAngleDeg = -30                               # Articulation angle in degrees
ArticulationAngleRad = np.deg2rad(ArticulationAngleDeg)    # Convert angle to radians for calculations
runAnimateArticulation = True                            # Flag to enable/disable animation of the articulation
ArticulationAxis = -10 



# ---- Generate grid for magnet positions ----

GridMargin = 10                                 # Margin from the edges to place magnets within the skin area
GridrowsMagnets = 2                             # Number of rows in the magnet grid
GridcolsMagnets = 3                             # Number of columns in the magnet grid
NMagnets = GridcolsMagnets * GridrowsMagnets    # Total number of magnets
MagnetSide = 1                                # Side length of each magnet (assuming square/cubic magnets)
CutMargin = GridMargin * 0.2                    # Additional margin for cutting or adjustment





# ---- Generate grid for sensor positions ----

SensorGridMargin = 10                                   # Margin from the edges to place sensors within the skin area
GridrowsSensors = 2                                    # Number of rows in the sensor grid
GridcolsSensors = 2                                     # Number of columns in the sensor grid
NSensors = GridcolsSensors * GridrowsSensors            # Total number of sensors
SensorCutMargin = SensorGridMargin * 0.2                # Additional margin for cutting or adjustment

# Sensor Dimensions
SensorLengthX = 3.0  
SensorLengthY = 2.0  
SensorLengthZ = 1  
SensorBoxTolerance = 0.1


# ---- Stretch test ----
displacement = -30.5


'''
------------------------------------------------------------------------------------
                          End of configurable parameters 
------------------------------------------------------------------------------------
'''

def generate_grid(length, width, margin, rows, cols):
    '''Generates a 2D grid of (x, y) points within a rectangle'''
    x = np.linspace(-(length - margin) / 2, (length - margin) / 2, cols)
    y = np.linspace(-(width - margin) / 2, (width - margin) / 2, rows)
    X, Y = np.meshgrid(x, y)
    return np.column_stack((X.ravel(), Y.ravel()))


def getBoxroiCoords(length, width, tolerance, marginX, marginZ):
    '''Generates the coordinates for creating a BoxROI'''
    max_x =  length/2 + tolerance  
    min_x = -(marginX + tolerance) 
    max_y =  width/2 + tolerance 
    min_y = -(marginZ + width/2 + tolerance) 
    max_z =  tolerance 
    min_z = -tolerance 
    
    return [max_x, max_y, max_z, min_x, min_y, min_z]


# ---- Generate grid points on the XY plane for magnets and sensors----
MagnetGridPoints = generate_grid(MagneticSkinLength, MagneticSkinWidth, GridMargin,  GridrowsMagnets,  GridcolsMagnets)
SensorGridPoints = generate_grid(MagneticSkinLength, MagneticSkinWidth, GridMargin, GridrowsSensors, GridcolsSensors)


# ---- Magnets and Sensors centers 3D coordinates ---- 
MagnetCenters = [[px, py, 2*MagneticSkinHeight/3] for px, py in MagnetGridPoints] 
SensorCenters = [[px, py,  MagneticSkinHeight/3] for px, py in SensorGridPoints]  # - ArticulationAxis in z axis for Simulationthumbfinger  





# ---- Rigid Articulation center 3D coordinates ---- 
rigidArticulationCenter =  np.array([[-MagneticSkinLength/2, 0, 0]])


# ---- Rigid center 3D coordinates ----
rigidObjects = np.vstack([rigidArticulationCenter, MagnetCenters, SensorCenters])
rigidObjects = rigidObjects.tolist()


# ---- Fixed BoxROI coordinates ----

# Defines a fixed region of interest (ROI) box with margins around the magnetic skin (rigid)
BoxROIFixCoords = getBoxroiCoords(MagneticSkinLength, MagneticSkinWidth, BoxROITolerance, marginX = -13, marginZ = 0) 

# Defines a rigidified region box on the opposite side (articulation)
BoxROIFixCoords1 = getBoxroiCoords(-MagneticSkinLength, -MagneticSkinWidth, BoxROITolerance, marginX = 0, marginZ = 0) 

# Rigidified region for the thumb finger
BoxROIFixCoordsThumb = getBoxroiCoords(-MagneticSkinLength, -MagneticSkinWidth, BoxROITolerance, marginX = -10, marginZ = 0) 




# ---- BoxROI coordinates for sensors ----

SensorBoxCoords = []

for point in SensorCenters:
    px, py, pz = point

    box = [
        px - (SensorLengthX / 2 + SensorBoxTolerance),  # min X
        py - (SensorLengthY / 2 + SensorBoxTolerance),  # min Y
        pz - (SensorLengthZ / 2 + SensorBoxTolerance),  # min Z
        px + (SensorLengthX / 2 + SensorBoxTolerance),  # max X
        py + (SensorLengthY / 2 + SensorBoxTolerance),  # max Y
        pz + (SensorLengthZ / 2 + SensorBoxTolerance)   # max Z
    ]
    SensorBoxCoords.append(box)



# ---- BoxROI coordinates for magnets ----
MagnetBoxCoords = []

# Add a BoxROI for each magnet in the grid, defining a bounding box around each magnet center
for point in MagnetCenters :
    px, py, pz = point  
    
    box = [
        px - (MagnetSide / 2 + MagnetBoxTolerance),  # min X coordinate of the magnet box
        py - (MagnetSide / 2 + MagnetBoxTolerance),  # min Y coordinate
        pz - (MagnetSide / 2 + MagnetBoxTolerance),  # min Z coordinate
        px + (MagnetSide / 2 + MagnetBoxTolerance),  # max X coordinate
        py + (MagnetSide / 2 + MagnetBoxTolerance),  # max Y coordinate
        pz + (MagnetSide / 2 + MagnetBoxTolerance)   # max Z coordinate
    ]    
    # Append the bounding box for this magnet to the list
    MagnetBoxCoords.append(box)


rigidObjectsBoxCoordsThumb = np.vstack([ [BoxROIFixCoordsThumb], MagnetBoxCoords, SensorBoxCoords])
rigidObjectsBoxCoordsThumb = rigidObjectsBoxCoordsThumb.tolist()
rigidObjectsBoxCoords = np.vstack([ [BoxROIFixCoords1], MagnetBoxCoords, SensorBoxCoords])
rigidObjectsBoxCoords = rigidObjectsBoxCoords.tolist()


# --- IndexPairs for SubsetMultiMapping ---
IndexPairs = [0, 1]  # Fixed region mapping
for i in range(NMagnets + NSensors):
    IndexPairs.extend([1, i])  # Map each magnet
# print(IndexPairs)
# # ---- Index for Sensor Centers ---- 
# indexPerPointSensor = []
# for center in SensorCenters:
#     if center[0] > MagnetBoxTolerance:
#         index = 0
#     else:
#         index = 1
#     indexPerPointSensor.append(index)     