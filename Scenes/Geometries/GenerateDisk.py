#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 27 15:49:56 2025

@author: benjamin
"""

import gmsh
from Constants import *

gmsh.initialize()
gmsh.model.add("DiskModel")

factory = gmsh.model.occ

disk = factory.addDisk(0, 0, 0, DiskRadius, DiskRadius)
disk = factory.extrude([(2, disk)], 0, 0, 1) 
factory.synchronize()

gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", 0.4)
gmsh.model.mesh.generate(2)

gmsh.write("Disk.stl")

gmsh.fltk.run()
gmsh.finalize()


