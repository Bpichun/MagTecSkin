#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 12 16:11:36 2025

@author: benjamin
"""

from Constants import *
import gmsh

factory = gmsh.model.occ
launchGUI = gmsh.fltk.run
synchronize = factory.synchronize


gmsh.initialize()

P1Tag = factory.addPoint(SensorLength/2, SensorWidth/2, 0)
P2Tag = factory.addPoint(-SensorLength/2, SensorWidth/2, 0)
P3Tag = factory.addPoint(-SensorLength/2, -SensorWidth/2, 0)
P4Tag = factory.addPoint(SensorLength/2, -SensorWidth/2, 0)

L1Tag = factory.addLine(P1Tag, P2Tag)
L2Tag = factory.addLine(P2Tag, P3Tag)
L3Tag = factory.addLine(P3Tag, P4Tag)
L4Tag = factory.addLine(P4Tag, P1Tag)

WireTag = factory.addWire([L1Tag,L2Tag,L3Tag,L4Tag])
SurfaceTag = factory.addPlaneSurface([WireTag])



ExtrudeOut = factory.extrude([(2, SurfaceTag)], 0, 0, SensorHeight)

BoxDimTag = ExtrudeOut[1]

# MagnetTag1 = factory.addBox(MagnetPosition[0]-MagnetSide/2, MagnetPosition[1]-MagnetSide/2,MagnetPosition[2]-MagnetSide/2, MagnetSide, MagnetSide, MagnetSide)
# MagnetDimTag1 = (3,MagnetTag1)
# factory.cut([BoxDimTag],[MagnetDimTag1] )

MagnetTags = []
for box in MagnetBoxCoords[1:]:           #MagnetBoxCoords[1:]
    x, y, z, dx, dy, dz = box
    dx = dx - x
    dy = dy - y
    dz = dz - z
    tag = factory.addBox(x, y, z, dx, dy, dz)
    MagnetTags.append((3, tag))
factory.cut([BoxDimTag], MagnetTags)




# cutTag1 = factory.addBox(-CutMargin/2, -SensorWidth/2, 1, CutMargin, SensorWidth, SensorHeight)
# cutDimTag1 = (3, cutTag1)
# factory.cut([BoxDimTag], [cutDimTag1])


# Corte para articulacion
# factory.cut([BoxDimTag], CutCoords)
# factory.cut([BoxDimTag], MagnetTags)



cutY = -SensorWidth / 2  

# Triangulo en XZ
T1 = factory.addPoint(-1, -SensorWidth / 2 , 0)  
T2 = factory.addPoint(0, -SensorWidth / 2 , 1)  
T3 = factory.addPoint(1, -SensorWidth / 2 , 0)   

LT1 = factory.addLine(T1, T2)
LT2 = factory.addLine(T2, T3)
LT3 = factory.addLine(T3, T1)

triangle_wire = factory.addWire([LT1, LT2, LT3])
triangle_surface = factory.addPlaneSurface([triangle_wire])

extruded_tri = factory.extrude([(2, triangle_surface)], 0, SensorWidth, 0)  
triangle_vol = extruded_tri[1]
factory.cut([BoxDimTag], [triangle_vol])




gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", 0.1)
synchronize()   
gmsh.model.mesh.generate(2)
gmsh.write("Sensor.stl")
# gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()

synchronize()
launchGUI()

gmsh.model.mesh.clear()
gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", 0.35)
gmsh.model.mesh.generate(3)
synchronize()
launchGUI()

gmsh.write("Sensor.vtk")
print(f"ExtrudeOut: {ExtrudeOut}")
synchronize()
launchGUI()

