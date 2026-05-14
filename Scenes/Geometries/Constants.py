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
# ---- Magnetic Skin physical parameters ----

# Physical dimensions of the MagneticSkin.
MagneticSkinHeight = 4     
MagneticSkinWidth = 20      
MagneticSkinLength = 50     

# Mechanical properties of the sensor material (used for physical deformation simulations).
PoissonRatio = 0.4            # Poisson ratio of the material
YoungsModulus = 60000        # Youngs modulus (material stiffness)


#Small characteristic length factor = finer mesh
SurfaceMeshCharacteristicLength = 0.1    # finer mesh for surfaces (2D)
VolumeMeshCharacteristicLength = 0.23    # coarser mesh for volume (3D)

#Magnet Dimension
MagnetSide = 1          

# Sensor Dimensions
SensorLength = 3.0  
SensorWidth = 2.0  
SensorHeigth = 1  


Use_Parametric = True
GridMargin = 10                                 # Margin from the edges to place magnets within the skin area


# ---- Generate grid for magnets and sensors positions  ----
#If Use_Parametric == True
GridrowsMagnets = 1                            # Number of rows in the magnet grid
GridcolsMagnets = 2                            # Number of columns in the magnet grid
GridrowsSensors = 1                            # Number of rows in the sensor grid    (X)
GridcolsSensors = 2                            # Number of columns in the sensor grid (Y)



'''--- External parameters ---'''
# ---- Indenter Parameters (SphereROI) ----
indenterRadius = 2                                    # Radius of the sphere used as the indenter
indenterPosition = [0, -5, MagneticSkinHeight]        # Initial position of the indenter 
indenterMotion = True                               # Enables or disables motion of the indenter
identadorForce = [0, 0, -300000]                      # Total force applied by the indenter 

# Articulation parameters
ArticulationAngleDeg = -10                            # Articulation angle in degrees
runAnimateArticulation = True                   # Flag to enable/disable animation of the articulation
ArticulationAxis = -10 

# Magnetic moment magnitude
mu_magnitude = 4.627195188680999e-08   

# ---- Stretch test ----
displacement = -0.5

# ---- Force Tests ----

forces = {
    "NormalForce": True,
    "ShearForce": False,
}

ForceTest = next((name for name, active in forces.items() if active), None)




'''
------------------------------------------------------------------------------------
                          End of configurable parameters 
------------------------------------------------------------------------------------
'''

# Extra tolerance around each rigid.
BoxTolerance = 0.1
ArticulationAngleRad = np.deg2rad(ArticulationAngleDeg)    


def generate_grid(length, width, margin, rows, cols):
    '''Generates a 2D grid of (x, y) points within a rectangle'''
    x = np.linspace(-(length - margin) / 2, (length - margin) / 2, cols)
    y = np.linspace(-(width - margin) / 2, (width - margin) / 2, rows)
    X, Y = np.meshgrid(x, y)
    return np.column_stack((X.ravel(), Y.ravel()))


def getBoxroiCoords(centers, lengths, tolerance):
    '''Generates the coordinates for creating a BoxROI'''
    lx, ly, lz = lengths
    boxes = []

    for px, py, pz in centers:
        box = [
            px - (lx / 2 + tolerance),
            py - (ly / 2 + tolerance),
            pz - (lz / 2 + tolerance),
            px + (lx / 2 + tolerance),
            py + (ly / 2 + tolerance),
            pz + (lz / 2 + tolerance)
        ]
        boxes.append(box)
    return boxes


# ---- Generate grid points on the XY plane for magnets and sensors----
MagnetGridPoints = generate_grid(MagneticSkinLength, MagneticSkinWidth, GridMargin,  GridrowsMagnets,  GridcolsMagnets)
SensorGridPoints = generate_grid(MagneticSkinLength, MagneticSkinWidth, GridMargin, GridrowsSensors, GridcolsSensors)


# ---- Magnets and Sensors centers 3D coordinates ---- 
MagnetCenters = [[px, py, 3*MagneticSkinHeight/4] for px, py in MagnetGridPoints] 
SensorCenters = [[px, py,  SensorHeigth*0.75] for px, py in SensorGridPoints]  # - ArticulationAxis in z axis for Simulationthumbfinger  


NMagnets = len(MagnetCenters)    
NSensors = len(SensorCenters)


# ---- Rigid Articulation center 3D coordinates ---- 
rigidArticulationCenter =  np.array([[-MagneticSkinLength/2, 0, 0]])


# ---- Rigid center 3D coordinates ----
rigidObjects = np.vstack([rigidArticulationCenter, MagnetCenters, SensorCenters])
rigidObjects = rigidObjects.tolist()


# ---- Fixed BoxROI coordinates ----

# Defines a fixed region of interest (ROI) box with margins around the magnetic skin (rigid)
BoxROIFixCoords = getBoxroiCoords(centers = [[MagneticSkinLength/2.1, 0, 0]] , #3.5
                                  lengths = [MagneticSkinLength/2.3, MagneticSkinWidth, BoxTolerance], #2.3
                                  tolerance = BoxTolerance)

print("BoxROIFixCoords:", BoxROIFixCoords)
# Defines a rigidified region box on the opposite side (articulation)
BoxROIFixCoordsArt = getBoxroiCoords(centers = [[-MagneticSkinLength/4  , 0, 0]] , 
                                     lengths = [MagneticSkinLength/2, MagneticSkinWidth, BoxTolerance],
                                     tolerance = BoxTolerance)


# Rigidified region for the thumb finger
BoxROIFixCoordsThumb = getBoxroiCoords(centers = [[-MagneticSkinLength/6.5, 0, 0]] , 
                                       lengths = [MagneticSkinLength/1.4, MagneticSkinWidth, BoxTolerance],
                                       tolerance = BoxTolerance)

print("BoxROIFixCoordsThumb:", BoxROIFixCoordsThumb)
# ---- BoxROI coordinates for sensors and magnets----
SensorBoxCoords = getBoxroiCoords(centers = SensorCenters, 
                                  lengths = (SensorLength, SensorWidth, SensorHeigth), 
                                  tolerance = BoxTolerance)

MagnetBoxCoords = getBoxroiCoords(centers = MagnetCenters, 
                                  lengths = (MagnetSide, MagnetSide, MagnetSide), 
                                  tolerance = BoxTolerance)



ObjectsBoxCoords = np.vstack([ MagnetBoxCoords, SensorBoxCoords])
print(ObjectsBoxCoords)
rigidObjectsBoxCoordsThumb = np.vstack([BoxROIFixCoordsThumb, MagnetBoxCoords, SensorBoxCoords])
rigidObjectsBoxCoordsThumb = rigidObjectsBoxCoordsThumb.tolist()

rigidObjectsBoxCoords = np.vstack([ BoxROIFixCoordsArt, MagnetBoxCoords, SensorBoxCoords])
rigidObjectsBoxCoords = rigidObjectsBoxCoords.tolist()


# --- IndexPairs for SubsetMultiMapping ---
'''Format: [source_index, DOF_index_in_source, source_index, DOF_index_in_source, ...]'''

IndexPairs = [[0, 1]] + [[1, i] for i in range(NMagnets)] # Mapping indices: [Fixed region, first object, ...]
IndexPairs= [idx for pair in IndexPairs for idx in pair]  # Map each Magnets
IndexPairsMagnets = np.copy(IndexPairs)
# ---- Add Index for Sensor Centers ---- 
indexPerPointSensor = []
indexSubSetMapSensor = []
for i, center in enumerate(SensorCenters):
    if center[0] > BoxTolerance:
        index = 0
    else:
        index = 1
    indexPerPointSensor.append((index))
    indexSubSetMapSensor.append((2, i))
    # indexSubSetMapSensor.append((index, i))

IndexPairs += [idx for pair in indexSubSetMapSensor for idx in pair]  # Map each Sensors

# indexSubSetMapSensor.append((index, i))
print("IndexPairs:", IndexPairs)

print(rigidObjects[1:])