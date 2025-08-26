#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 12 16:11:36 2025

@author: benjamin
"""


"""
This script uses Gmsh to model a 3D box-shaped sensor (parallelepiped), 
performs boolean cuts to create cavities for magnets and a triangular slot, 
and generates mesh files in STEP, STL, and VTK formats.
"""


from Constants import *
import gmsh


# Access to CAD operations (OpenCascade)
factory = gmsh.model.occ

# Shortcut to open the graphical user interface
launchGUI = gmsh.fltk.run
synchronize = factory.synchronize


# Initialize the Gmsh environment
gmsh.initialize()


# Creation of the sensor base (rectangle in the XY plane)
P1Tag = factory.addPoint(MagneticSkinLength/2, MagneticSkinWidth/2, 0)
P2Tag = factory.addPoint(-MagneticSkinLength/2, MagneticSkinWidth/2, 0)
P3Tag = factory.addPoint(-MagneticSkinLength/2, -MagneticSkinWidth/2, 0)
P4Tag = factory.addPoint(MagneticSkinLength/2, -MagneticSkinWidth/2, 0)

L1Tag = factory.addLine(P1Tag, P2Tag)
L2Tag = factory.addLine(P2Tag, P3Tag)
L3Tag = factory.addLine(P3Tag, P4Tag)
L4Tag = factory.addLine(P4Tag, P1Tag)

WireTag = factory.addWire([L1Tag, L2Tag, L3Tag, L4Tag])       # Rectangle contour
SurfaceTag = factory.addPlaneSurface([WireTag])               # Rectangle surface


# Extrusion of the rectangle to generate the 3D volume of the MagneticSkin

ExtrudeOut = factory.extrude([(2, SurfaceTag)], 0, 0, MagneticSkinHeight)  # Extrusion along Z-axis
BoxDimTag = ExtrudeOut[1]                                                  # ID of the created volume



# MagnetTag1 = factory.addBox(MagnetPosition[0]-MagnetSide/2, MagnetPosition[1]-MagnetSide/2,MagnetPosition[2]-MagnetSide/2, MagnetSide, MagnetSide, MagnetSide)
# MagnetDimTag1 = (3,MagnetTag1)
# factory.cut([BoxDimTag],[MagnetDimTag1] )

ObjectsBoxCoords = np.vstack([ MagnetBoxCoords, SensorBoxCoords])


boxroiTags = []
for box in ObjectsBoxCoords :           #MagnetBoxCoords[1:]
    x, y, z, dx, dy, dz = box
    dx = dx - x
    dy = dy - y
    dz = dz - z
    tag = factory.addBox(x, y, z, dx, dy, dz)
    boxroiTags .append((3, tag))
factory.cut([BoxDimTag], boxroiTags )




# cutTag1 = factory.addBox(-CutMargin/2, -MagneticSkinWidth/2, 1, CutMargin, MagneticSkinWidth, MagneticSkinHeight)
# cutDimTag1 = (3, cutTag1)
# factory.cut([BoxDimTag], [cutDimTag1])


# Corte para articulacion
# factory.cut([BoxDimTag], CutCoords)
# factory.cut([BoxDimTag], MagnetTags)



# cutY = -MagneticSkinWidth / 2  

# # Triangulo en XZ
# T1 = factory.addPoint(-1, -MagneticSkinWidth / 2 , 0)  
# T2 = factory.addPoint(0, -MagneticSkinWidth / 2 , 1)  
# T3 = factory.addPoint(1, -MagneticSkinWidth / 2 , 0)   

# LT1 = factory.addLine(T1, T2)
# LT2 = factory.addLine(T2, T3)
# LT3 = factory.addLine(T3, T1)

# triangle_wire = factory.addWire([LT1, LT2, LT3])
# triangle_surface = factory.addPlaneSurface([triangle_wire])

# extruded_tri = factory.extrude([(2, triangle_surface)], 0, MagneticSkinWidth, 0)  
# triangle_vol = extruded_tri[1]
# factory.cut([BoxDimTag], [triangle_vol])

synchronize()             # Synchronize the CAD model with the Gmsh model
launchGUI()               # Open GUI to review the geometry


gmsh.write("MagneticSkin.step")    # Save geometry in STEP format (CAD)

gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", SurfaceMeshCharacteristicLength)  # Small characteristic length = finer mesh
synchronize()   
gmsh.model.mesh.generate(2)                                     # Generate surface mesh (2D)
gmsh.write("MagneticSkin.stl")                                        # Save as STL

synchronize()
launchGUI()

gmsh.model.mesh.clear()
gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", VolumeMeshCharacteristicLength)  # Mesh for volume
gmsh.model.mesh.generate(3)                                     # Generate volume mesh (3D)
synchronize()
launchGUI()


gmsh.write("MagneticSkin.vtk")      # Export 3D mesh 
print(f"ExtrudeOut: {ExtrudeOut}")
synchronize()