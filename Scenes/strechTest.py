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
# import SofaRuntime
# from stlib3.scene import Scene
# from splib3.animation import animate
# from scipy.spatial.transform import Rotation as R
import Geometries.Constants as Const
import rigidification  
import Sofa.Core
import Sofa.constants.Key as Key



# ---- Directory Paths ----
dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
path = os.path.dirname(os.path.abspath(__file__))+'/Meshes/'
MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

TempPath = os.path.dirname(os.path.abspath(__file__))+'/Temp/'


EdgeMargin = 2  

# --- right BoxROI para el borde derecho ---
BoxROIFixCoords = [
    Const.MagneticSkinLength / 2 + Const.BoxROITolerance, 
    Const.MagneticSkinWidth  / 2 + Const.BoxROITolerance, 
    Const.MagneticSkinHeight  + Const.BoxROITolerance,  
    Const.MagneticSkinLength / 2 - EdgeMargin,       
    -(Const.MagneticSkinWidth / 2 + Const.BoxROITolerance),
     - Const.BoxROITolerance  
]

# --- left BoxROI ---
BoxROIFixCoords1 = [
    -(Const.MagneticSkinLength / 2 - EdgeMargin),           
    Const.MagneticSkinWidth  / 2 + Const.BoxROITolerance,   
    Const.MagneticSkinHeight + Const.BoxROITolerance,       
    -(Const.MagneticSkinLength / 2 + Const.BoxROITolerance),
    -(Const.MagneticSkinWidth / 2 + Const.BoxROITolerance), 
    -Const.BoxROITolerance                                  
]

class ControlKeyboard(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.increment = 0.3
        # self.previousValue = [0, 0, 0]  # solo una “pierna”

    def onKeypressedEvent(self, e):
        grab_node = self.node.ExternalRefNode
        if grab_node is None:
            return  
        grab = grab_node.getObject('ExternalMO')
        if grab is None:
            # print("was not able to get MO")
            return

        # currentValue = np.array(grab.rest_position.value)
        currentValue = grab.translation
        print(f"currentValue: {currentValue}")

        # Movimientos según la tecla
        if e['key'] == Key.leftarrow:
            currentValue[0] = -self.increment
            # currentValue[:,0] = currentValue[:,0]-self.increment
        elif e['key'] == Key.rightarrow:
            currentValue[0] = self.increment
            # currentValue[:,0] = currentValue[:,0]+self.increment
        
            
        grab.translation.value = currentValue
        # grab.rest_position.value = currentValue
        grab.reinit()



class SinusoidalController(Sofa.Core.Controller):
    def __init__(self, node, amplitude=-2.5, frequency=0.5):
        super().__init__()
        self.node = node
        self.amplitude = amplitude  
        self.frequency = frequency   
        self.start_time = None

    def onAnimateBeginEvent(self, event):
        t = float(self.node.getRootContext().time.value)  
        if self.start_time is None:
            self.start_time = t
        t_rel = t - self.start_time


        displacement = self.amplitude * np.sin(2 * math.pi * self.frequency * t_rel*0.001)
        print(displacement )
        grab_node = self.node.ExternalRefNode
        if grab_node is None:
            return
        grab = grab_node.getObject('ExternalMO')
        if grab is None:
            return

        currentValue = list(grab.translation.value)
        currentValue[0] = displacement   

        grab.translation.value = currentValue
        grab.reinit()

class FixedController(Sofa.Core.Controller):
    def __init__(self, node, displacement=Const.displacement):
        super().__init__()
        self.node = node
        self.displacement = displacement  
        self.initialized = False

    def onAnimateBeginEvent(self, event):
        if self.initialized:
            return  

        grab_node = self.node.ExternalRefNode
        if grab_node is None:
            return
        grab = grab_node.getObject('ExternalMO')
        if grab is None:
            return

        currentValue = list(grab.translation.value)
        currentValue[0] = self.displacement  
        grab.translation.value = currentValue
        grab.reinit()

        self.initialized = True  



def createScene(rootNode):
                
    rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')
    rootNode.findData('gravity').value = [0, 0, -0] 
    rootNode.findData('dt').value = 0.02

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog='1', epsilon="1e-1", maxIterations="1000", tolerance="1e-5")
    #rootNode.addObject('QPInverseProblemSolver', printLog=False, epsilon="0.0001", maxIterations="1000", tolerance="1e-5")

    rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

    #rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    rootNode.addObject('BackgroundSetting', color='0.85 0.85 0.85')
    
    
    rootNode.addObject('LightManager')
    rootNode.addObject(
        "PositionalLight",
        name="light1",
        color="0.8 0.8 0.8",
        position=[0, 0, 50],
    )
    rootNode.addObject(
        "PositionalLight",
        name="light2",
        color="0.8 0.8 0.8",
        position=[0, 0, -50],
    )
    


    # ---- Paths to mesh files ----   
    VolumetricMeshPath = GeneratedMeshesPath + 'MagneticSkin.vtk'
    
    SurfaceMeshPath = GeneratedMeshesPath + 'MagneticSkin.stl'
    
    
    # rootNode.addObject(SinusoidalController(node=rootNode))
    rootNode.addObject(FixedController(rootNode))
    
    #----------------------
    # Goal Node
    #---------------------- 
 
    #----------------------
    # Rigidification - start
    #----------------------          
                   
    completeMesh = rootNode.addChild('completeMesh')
    completeMesh.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
    completeMesh.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    completeMesh.init()
    MeshTetra = completeMesh.addObject('MeshTopology', name="AllMesh", src='@loader')

    

    
    #---- Create BoxROIs ----
    Boxes = []
    for i in range(len(Const.MagnetFreeCenters)):
        boxTip = completeMesh.addObject('BoxROI', name='Tip'+str(i), box=[Const.MagnetBoxCoordstest[i]], drawBoxes=False, tetrahedra="@container.tetrahedra" , position="@container.position")
        Boxes.append(boxTip)
        boxTip.init()
      
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


    
    solverNode = rootNode.addChild("solverNode")
    solverNode.addObject('EulerImplicitSolver',rayleighStiffness="0.1", rayleighMass="0.1")
    solverNode.addObject('SparseLDLSolver',name='preconditioner')
    solverNode.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')


    RigidNode= solverNode.addChild('RigidNode')
     
    
    nominal_pose = [] 
    TipOrientation = [0, 0, 0, 1]       
    
    
    for center in Const.MagnetFreeCenters:
        CurrentPose = center + TipOrientation
        nominal_pose += CurrentPose
    RigidMO = RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=nominal_pose, 
                                  showObject=True, showObjectScale=2, showIndices=True) # orientation is 240 deg away from scene origin


    RigidifiedNode =  RigidNode.addChild('RigidifiedNode')   
    RigidifiedNode.addObject('MechanicalObject', name='RigidifiedMesh', position=pointsTip,
                             template='Vec3d', showObject=True, showObjectScale=4, showColor=1)       
    RigidifiedNode.addObject("RigidMapping", globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPoint)
    

    # ---- Deformable Node -----
    deformableNode = RigidifiedNode.addChild("deformableNode")
    deformableNode.addObject('PointSetTopologyContainer', position=pointsBody)
    deformableNode.addObject('MechanicalObject', name='DeformableMech', showObject = False, showObjectScale = 4)
    # deformableNode.addObject('BarycentricMapping', input='@model/tetras', output='@DeformableMech')
    

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
    model.addObject('BoxROI', name='BaseROI', box=BoxROIFixCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")              
    model.addObject('RestShapeSpringsForceField', points='@BaseROI.indices', stiffness='1e12')              
    model.addObject("SubsetMultiMapping",name="subsetMapping",template="Vec3d,Vec3d", input='@'+deformableNode.getPathName()+'/DeformableMech' + ' ' + '@'+RigidifiedNode.getPathName()+'/RigidifiedMesh' , output='@./tetras', indexPairs=indexPairs.tolist())
    model.addObject('BoxROI', name='EndROI', box=BoxROIFixCoords1, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")              
    model.addObject('RestShapeSpringsForceField', points='@EndROI.indices', stiffness='1e12', external_rest_shape="@ExternalRefNode/ExternalMO")              

    
    # ----------------------------------------
    # Visualization                          
    # ----------------------------------------
    
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
    modelVisu.addObject('BarycentricMapping')
    
    ExternalRefNode = rootNode.addChild("ExternalRefNode")
    ExternalRefNode.addObject('MechanicalObject', name='ExternalMO', template='Vec3', showObject=True, showObjectScale=15, showColor = [0,0,.7],
                   showIndices=False, showIndicesScale=4e-5,
                   position='@../solverNode/RigidNode/RigidifiedNode/deformableNode/model/EndROI.pointsInROI')

    return rootNode