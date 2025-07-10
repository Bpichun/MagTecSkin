#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 26 14:55:04 2025

@author: benjamin
"""


# ---- Import Libraries ----
import os
import numpy as np
import Sofa
import SofaRuntime
from stlib3.scene import Scene
from splib3.animation import animate
from scipy.spatial.transform import Rotation as R
import Geometries.Constants as Const
import rigidification  
import math 


dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
path = os.path.dirname(os.path.abspath(__file__))+'/Meshes/'
MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
TempPath = os.path.dirname(os.path.abspath(__file__))+'/Temp/'

def createScene(rootNode):
    
    SofaRuntime.importPlugin("ArticulatedSystemPlugin")

    pluginsList = [
        'ArticulatedSystemPlugin',
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Constraint.Lagrangian.Correction',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.Component.Constraint.Projective',
        'Sofa.Component.IO.Mesh',
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.Mapping.NonLinear',
        'Sofa.Component.Mass',
        'Sofa.Component.SolidMechanics.Spring',
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Constant',
        'Sofa.Component.Visual',
        'Sofa.GL.Component.Rendering3D',
        'Sofa.GUI.Component',
    ]


    scene = Scene(rootNode, plugins=pluginsList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.addObject('BackgroundSetting', color='0.8 0.8 0.8')
    scene.dt = 0.02
    scene.gravity = [0., -9810., 0.]
    # scene.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

    # ---- Fixed servoBody ----
    servoBody = scene.Simulation.addChild('ServoBody')
    servoBody.addObject('MechanicalObject', name='dofs', template='Rigid3',
                        position=[[0., 0., 0, 0., 0., 0., 1.]]) 
    servoBody.addObject('FixedProjectiveConstraint', indices=0)
    servoBody.addObject('UniformMass', totalMass=0.01)


    visual = servoBody.addChild('VisualModel')
    visual.addObject('MeshSTLLoader', name='loader', filename=GeneratedMeshesPath + 'pip.stl')
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 1.0], translation=[0, -7.5, -13])
    visual.addObject('RigidMapping', index=0)




    
    articulationAngle = scene.Simulation.addChild('Articulation')
    articulationAngle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]], rest_position=[[0]])
    articulationAngle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
    articulationAngle.addObject('UniformMass', totalMass=0.01)

    servoWheel = articulationAngle.addChild('ServoWheel')
    servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                          position=[[0., 0., 15., 0., 0., 0., 1.], [0., 0., 15, 0., 0., 0., 1.]], 
                          showObject = False, showObjectScale=15)
    
    
    
    servoWheel.addObject('ArticulatedSystemMapping', input1='@../dofs', input2='@../../ServoBody/dofs', output='@./')
    
    
    
    visual1 = servoWheel.addChild('VisualModel1')
    visual1.addObject('MeshSTLLoader', name='loader', filename= GeneratedMeshesPath + 'fingertip.stl')
    visual1.addObject('MeshTopology', src='@loader')
    visual1.addObject('OglModel', color=[0.4, 0.45, 0.5, 1.0], translation=[-0, 0, 0])
    visual1.addObject('RigidMapping', index=1)
    
    # --- Define Articulation center ----
    articulationCenter = articulationAngle.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0, 0, 0], posOnChild=[0, 0,0])
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)
    articulationAngle.addObject('ArticulatedHierarchyContainer')
    
    
    

    servoWheel.dofs.showObject = True
    
    
    def animation(target, factor):
        angle_start = -0   
        angle_end = -115  
        angle_start = np.deg2rad(angle_start)
        angle_end = np.deg2rad(angle_end)
        angle = angle_start + (angle_end - angle_start) * 0.5 * (1 - math.cos(2 * math.pi * factor))
        target.dofs.rest_position[0][0] = angle 
            
        
    animate(animation, {'target': articulationAngle}, duration=10., mode='loop')

    return scene
