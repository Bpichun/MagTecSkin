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
MagneticSkinHeight = 3      # Height (Z-axis)
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
indenterRadius = 2.5                                       # Radius of the sphere used as the indenter
indenterPosition = [0, 0, MagneticSkinHeight / 2]           # Initial position of the indenter (on the top surface)
indenterMotion = True                                     # Enables or disables motion of the indenter
identadorForce = [0, 0, -100000]                           # Total force applied by the indenter (negative Z direction)

# Articulation parameters
ArticulationAngleDeg = -30                               # Articulation angle in degrees
ArticulationAngleRad = np.deg2rad(ArticulationAngleDeg)    # Convert angle to radians for calculations
runAnimateArticulation = True                            # Flag to enable/disable animation of the articulation
ArticulationAxis = -10 



# ---- Generate grid for magnet positions ----

GridMargin = 10                                 # Margin from the edges to place magnets within the skin area
GridrowsMagnets = 3                             # Number of rows in the magnet grid
GridcolsMagnets = 5                             # Number of columns in the magnet grid
NMagnets = GridcolsMagnets * GridrowsMagnets    # Total number of magnets
MagnetSide = 1                                # Side length of each magnet (assuming square/cubic magnets)
CutMargin = GridMargin * 0.2                    # Additional margin for cutting or adjustment


# ---- Generate grid for sensor positions ----

SensorGridMargin = 10                                   # Margin from the edges to place sensors within the skin area
GridrowsSensors = 3                                     # Number of rows in the sensor grid
GridcolsSensors = 5                                     # Number of columns in the sensor grid
NSensors = GridcolsSensors * GridrowsSensors            # Total number of sensors
SensorCutMargin = SensorGridMargin * 0.2                # Additional margin for cutting or adjustment



# ---- Stretch test ----
displacement = -30.5


'''
------------------------------------------------------------------------------------
                          End of configurable parameters 
------------------------------------------------------------------------------------
'''



# ---- Generate grid points on the XY plane for magnets----

# Create evenly spaced points along the length (X-axis) and  width (Y-axis), centered within the magnetic skin.
x = np.linspace(-(MagneticSkinLength - GridMargin) / 2, (MagneticSkinLength - GridMargin) / 2, GridcolsMagnets)
y = np.linspace(-(MagneticSkinWidth - GridMargin) / 2, (MagneticSkinWidth - GridMargin) / 2, GridrowsMagnets)

# Generate a 2D grid mesh of points from the x and y coordinates
X, Y = np.meshgrid(x, y)

# ---- Flatten the grid points into a list of (x, y) coordinate pairs ----

MagnetGridPoints = np.column_stack((X.ravel(), Y.ravel()))



# ---- Magnets centers 3D coordinates ---- 
MagnetCenters = []
MagnetCenters.append([-7.5, 0, 0])                  #quito para strech test
MagnetCenters += [[px, py, MagneticSkinHeight / 2] for px, py in MagnetGridPoints] 
MagnetFreeCenters = MagnetCenters[1:]


# ---- Generate grid points on the XY plane for sensors ----

# Create evenly spaced points along X-axis
xs = np.linspace(-(MagneticSkinLength - SensorGridMargin) / 2, 
                  (MagneticSkinLength - SensorGridMargin) / 2, 
                  GridcolsSensors)

# Create evenly spaced points along Y-axis
ys = np.linspace(-(MagneticSkinWidth - SensorGridMargin) / 2, 
                  (MagneticSkinWidth - SensorGridMargin) / 2, 
                  GridrowsSensors)

# Generate 2D grid mesh
Xs, Ys = np.meshgrid(xs, ys)

# Flatten the grid into list of (x, y) pairs
SensorGridPoints = np.column_stack((Xs.ravel(), Ys.ravel()))

# ---- Sensors centers 3D coordinates ---- 
SensorCenters = [[px, py,  -MagneticSkinHeight/2]  # - ArticulationAxis in z axis for Simulationthumbfinger
                  for px, py in SensorGridPoints]





# ---- Index for Sensor Centers ---- 
indexPerPointSensor = []
for center in SensorCenters:
    if center[0] > MagnetBoxTolerance:
        index = 0
    else:
        index = 1
    indexPerPointSensor.append(index)



# ---- Fixed BoxROI coordinates ----

# Defines a fixed region of interest (ROI) box with margins around the magnetic skin (rigid)
BoxROIFixCoords = [
    MagneticSkinLength / 2 + BoxROITolerance,    # max X coordinate 
    MagneticSkinWidth  / 2 + BoxROITolerance,    # max Y coordinate 
    BoxROITolerance,                             # max Z coordinate 
    -(-14 + BoxROITolerance),             # min X coordinate             
    -(MagneticSkinWidth / 2 + BoxROITolerance),  # min Y coordinate 
    -BoxROITolerance                             # min Z coordinate 
]

# Defines a rigidified region box on the opposite side (articulation)
BoxROIFixCoords1 = [
    -(MagneticSkinLength / 2 + BoxROITolerance ),   # min X
    -(MagneticSkinWidth  / 2 + BoxROITolerance),   # min Y
    -MagneticSkinHeight  / 2 - BoxROITolerance,    # min Z
    0,                           # max X
    MagneticSkinWidth    / 2 + BoxROITolerance,    # max Y
    BoxROITolerance                                 # max Z
]




BoxROIFixCoordsThumb = [
    -(MagneticSkinLength / 2 + BoxROITolerance ),   
    -(MagneticSkinWidth  / 2 + BoxROITolerance),   
    -MagneticSkinHeight  / 2 - BoxROITolerance,    
    10-BoxROITolerance,                                             #Modificando para thumbfinger
    MagneticSkinWidth    / 2 + BoxROITolerance,       
    BoxROITolerance                                
]



# ---- BoxROI coordinates for magnets ----
MagnetBoxCoords = []

# Add the fixed rigidified region to the list of magnet boxes
MagnetBoxCoords.append(BoxROIFixCoords1)                           #sacar tambien

# Add a BoxROI for each magnet in the grid, defining a bounding box around each magnet center
for point in MagnetGridPoints:
    px, py = point
    pz = MagneticSkinHeight / 2  
    
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

MagnetBoxCoordstest = MagnetBoxCoords[1:]


MagnetBoxCoordsThumb = MagnetBoxCoords.copy()  
MagnetBoxCoordsThumb[0] = BoxROIFixCoordsThumb

# --- IndexPairs for SubsetMultiMapping ---
IndexPairs = [0, 1]  # Fixed region mapping
for i in range(NMagnets):
    IndexPairs.extend([1, i])  # Map each magnet
     