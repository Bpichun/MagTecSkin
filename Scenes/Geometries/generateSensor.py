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
for box in MagnetBoxCoords:
    x, y, z, dx, dy, dz = box
    dx = dx - x
    dy = dy - y
    dz = dz - z
    tag = factory.addBox(x, y, z, dx, dy, dz)
    MagnetTags.append((3, tag))

factory.cut([BoxDimTag], MagnetTags)


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
# synchronize()
# launchGUI()

