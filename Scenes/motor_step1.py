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
    visual.addObject('MeshSTLLoader', name='loader', filename='/home/benjamin/Downloads/pip1.stl')
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 1.0])
    visual.addObject('RigidMapping', index=0)




    
    articulationAngle = scene.Simulation.addChild('Articulation')
    articulationAngle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]], rest_position=[[0]])
    articulationAngle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
    articulationAngle.addObject('UniformMass', totalMass=0.01)

    servoWheel = articulationAngle.addChild('ServoWheel')
    servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                          position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]], 
                          showObject = False, showObjectScale=15)
    
    
    
    servoWheel.addObject('ArticulatedSystemMapping', input1='@../dofs', input2='@../../ServoBody/dofs', output='@./')
    
    
    
    visual1 = servoWheel.addChild('VisualModel1')
    # visual1.addObject('MechanicalObject', name='visuMO',template='Rigid3', position=[[0.0, 10, 0.0, 0, 0, 0, 1]], showObject = True, showObjectScale=15)
    # visual1.addObject('RigidMapping', index=1)
    visual1.addObject('MeshSTLLoader', name='loader', filename='/home/benjamin/Downloads/parte2.stl')
    visual1.addObject('MeshTopology', src='@loader')
    visual1.addObject('OglModel', color=[0.4, 0.45, 0.5, 1.0], translation=[-35, -30, 5])
    visual1.addObject('RigidMapping', index=0)
    
    # --- Define Articulation center ----
    articulationCenter = articulationAngle.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0, 0, -0], posOnChild=[0, 0, -0])
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)
    articulationAngle.addObject('ArticulatedHierarchyContainer')
    
    
    

    servoWheel.dofs.showObject = True
    
    

    
    def animation(target, factor):
        # target.dofs.position[0][0] = math.cos(factor * 2 * math.pi)
        target.dofs.rest_position[0][0] = math.cos(factor * 2 * math.pi)

        # print("Articulation angle:", target.dofs.position[0][0])
        # print("ServoWheel pose[1]:", servoWheel.dofs.position[1])
    animate(animation, {'target': articulationAngle}, duration=10., mode='loop')

    return scene
