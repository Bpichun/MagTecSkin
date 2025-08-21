#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 19 09:29:11 2025

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


# ---- Directory Paths ----
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
    scene.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')




    # ---- Paths to mesh files ----   
    VolumetricMeshPath = GeneratedMeshesPath + 'MagneticSkin.vtk'
    
    SurfaceMeshPath = GeneratedMeshesPath + 'MagneticSkin.stl'
    
    
    #----------------------
    # Goal Node
    #---------------------- 
 
    #----------------------
    # Rigidification - start
    #----------------------          
                   
    completeMesh = scene.addChild('completeMesh')
    completeMesh.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
    completeMesh.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    completeMesh.init()
    
    
    
    #---- Create BoxROIs ----
    Boxes = []
    for i in range(len(Const.MagnetBoxCoords)):
        boxTip = completeMesh.addObject('BoxROI', name='Tip'+str(i), box=[Const.MagnetBoxCoords[i]], drawBoxes=True, tetrahedra="@container.tetrahedra" , position="@container.position")
        Boxes.append(boxTip)
        boxTip.init()
      
    MeshTetra = completeMesh.addObject('MeshTopology', name="AllMesh", src='@loader')
    positionAllPoints = MeshTetra.findData('position').value;
    nbPoints = len(positionAllPoints)
    
    
    IndicesWithRigidIdx = np.empty((0,2), dtype=int)
    
    for (i,Box) in enumerate(Boxes):
        IndicesNP = np.array(Box.indices.value, dtype=int)
        NPoints = len(IndicesNP)
        RigidIdx = np.ones(NPoints,dtype=int)*i
        CurrentIndicesWithRigidIdx = np.append(IndicesNP.reshape((NPoints,1)), RigidIdx.reshape((NPoints,1)),1)
        IndicesWithRigidIdx = np.append(IndicesWithRigidIdx, CurrentIndicesWithRigidIdx,0)
    

    IndicesWithRigidIdxSorted = np.sort(IndicesWithRigidIdx[:,0],0)
    SortedIdxs = np.argsort(IndicesWithRigidIdx[:,0],0)
    SortedRigidIdxs = IndicesWithRigidIdx[:,1][SortedIdxs]

    indicesTip = IndicesWithRigidIdxSorted.tolist()
    
    rigidBlocks = [IndicesWithRigidIdxSorted.tolist()] 
    
    DeformableIndicesTotal = []    
    
    
    

    for i in range(nbPoints):
        if i not in indicesTip:
            DeformableIndicesTotal.append(i)                                 

    freeBlocks = np.sort(DeformableIndicesTotal)    
    IdxsOrderedFreeBlocks = np.argsort(DeformableIndicesTotal)    
    indexPairs = np.array(rigidification.fillIndexPairs(nbPoints,freeBlocks,rigidBlocks))
    NPPointsDeformable = positionAllPoints[DeformableIndicesTotal,:]   
    NPSortedPointsDeformable = NPPointsDeformable[IdxsOrderedFreeBlocks, :]
    PointsDeformable = NPSortedPointsDeformable.flatten().tolist()
    pointsBody = PointsDeformable
    #deformablePoints = pointsBody
       
    pointsTip = np.array(positionAllPoints[indicesTip,:]).flatten().tolist()                                                 
    rigidIndexPerPoint = SortedRigidIdxs.tolist()


    
    # ----------------------------------------
    #            Articulation                           
    # ----------------------------------------
  
    
    # ---- Fixed servoBody ----
    servoBody = scene.Simulation.addChild('ServoBody')
    servoBody.addObject('MechanicalObject', name='dofs', template='Rigid3',
                        position=[[0., 0., Const.MagneticSkinHeight/2, 0., 0., 0., 1.]]) 
    servoBody.addObject('FixedProjectiveConstraint', indices=0)
    servoBody.addObject('UniformMass', totalMass=0.01)


    # ---- Articulation angle: 1 DOF rotation  ----
    articulationAngle = scene.Simulation.addChild('Articulation')
    articulationAngle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]], rest_position=[[0]])
    articulationAngle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
    articulationAngle.addObject('UniformMass', totalMass=0.01)



    # ---- ServoWheel ----
    servoWheel = articulationAngle.addChild('ServoWheel')
    servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                         position=[[0., 0., Const.MagneticSkinHeight/2., 0., 0., 0., 1.], 
                                   [0., 0., Const.MagneticSkinHeight/2., 0., 0., 0., 1.]],
                         showObject = True, showObjectScale=4)
    

    
    # ---- Mapping articulation control to servo and body -----
    servoWheel.addObject('ArticulatedSystemMapping', input1='@../dofs', input2='@../../ServoBody/dofs', output='@./')

   

    RigidNode= servoWheel.addChild('RigidNode')
     
    
    nominal_pose = [] 
    TipOrientation = [0, 0, 0, 1]       
    
    
    for center in Const.MagnetCenters:
        CurrentPose = center + TipOrientation
        nominal_pose += CurrentPose
    RigidMO = RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=nominal_pose, 
                                  showObject=True, showObjectScale=2, showIndices=True) # orientation is 240 deg away from scene origin
    
    # RigidNode.addObject("RigidMapping", input="@../dofs", output="@RigidMesh", index = 0)

  
    nominal_pose1 = [] 
    
    for center in Const.MagnetFreeCenters:
        CurrentPose = center + TipOrientation
        nominal_pose1 += CurrentPose
    
    freeCenter = scene.Simulation.addChild("freeCenter")
    freeCenter.addObject("MechanicalObject", name="dofs", template="Rigid3",
                         position=nominal_pose1,
                         showObject=False, showObjectScale=0)
    freeCenter.addObject("UniformMass", totalMass=0.01)
    freeCenter.addObject("EulerImplicitSolver")
    freeCenter.addObject("SparseLDLSolver")
    freeCenter.addChild(RigidNode)
    
  
    
    RigidifiedNode =  RigidNode.addChild('RigidifiedNode')   
    RigidifiedNode.addObject('MechanicalObject', name='RigidifiedMesh', position=pointsTip,
                             template='Vec3d', showObject=True, showObjectScale=4, showColor=1)       
    RigidifiedNode.addObject("RigidMapping", globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPoint)
    
    
    
    # ---- SubsetMultiMapping to connect ServoWheel and freeCenter to rigid tips ----
    RigidNode.addObject('SubsetMultiMapping',
                          name="mapping",
                          input=['@../dofs', '@/Simulation/freeCenter/dofs'],
                          output='@./', indexPairs=Const.IndexPairs)
    
    
    
    # --- Define Articulation center ----
    articulationCenter = articulationAngle.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0], articulationIndex=0)
    articulationAngle.addObject('ArticulatedHierarchyContainer')
    
    
    
    # ---- Deformable Node -----
    deformableNode = RigidifiedNode.addChild("deformableNode")
    deformableNode.addObject('PointSetTopologyContainer', position=pointsBody)
    deformableNode.addObject('MechanicalObject', name='DeformableMech', showObject = True, showObjectScale = 4)
   
    
   
    model = deformableNode.addChild('model')
    RigidifiedNode.addChild(model)
    
    
    
    	#---- Heart ----
    model.addObject('EulerImplicitSolver', name='nodesolver')          
    model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', update_step='1')
    model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
    model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
    model.addObject('UniformMass', totalMass='0.09')
    model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=Const.PoissonRatio,  youngModulus=Const.YoungsModulus)               
    model.addObject('BoxROI', name='BaseROI', box=Const.BoxROIFixCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")            
    model.addObject('RestShapeSpringsForceField', points='@BaseROI.indices', stiffness='1e12')                
    model.addObject("SubsetMultiMapping",name="subsetMapping",template="Vec3d,Vec3d", input='@'+deformableNode.getPathName()+'/DeformableMech' + ' ' + '@'+RigidifiedNode.getPathName()+'/RigidifiedMesh' , output='@./tetras', indexPairs=indexPairs.tolist())
      
    
    
    # ----------------------------------------
    # Visualization                          
    # ----------------------------------------
    
    
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
    modelVisu.addObject('BarycentricMapping')





    return scene