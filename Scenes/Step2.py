#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 19 09:29:11 2025

@author: benjamin
"""

import os
import Sofa
import SofaRuntime
import numpy as np
from scipy.spatial.transform import Rotation as R 
#from stlib.physics.deformable import ElasticMaterialObject
#from stlib.physics.constraints import FixedBox
#from stlib.physics.collision import CollisionMesh
from stlib3.scene import Scene
from splib3.animation import animate
import Geometries.Constants as Const
import rigidification

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
path = os.path.dirname(os.path.abspath(__file__))+'/Meshes/'
MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
TempPath = os.path.dirname(os.path.abspath(__file__))+'/Temp/'


def CalcularB(Distancia_r_mm, Direccion_momento_magnetico,mu_mag_delGráfico): #
            # Definición de variables
            Distancia_r = np.array(Distancia_r_mm)
            length_r = np.linalg.norm(Distancia_r) 
            # print("length_r", length_r)
            r_hat = Distancia_r / length_r
            # print("r_hat",r_hat)
            
            Mu_hat = Direccion_momento_magnetico  # Dirección del momento magnético (vector unitario)

        # ······························································
            # Rotación (Definir rotación en CalcularB(x,Direccion_momento_magnetico,x), no comentandolo)··········································
        # ······························································
            # Mu_hat = mu/np.linalg.norm(mu)
            # mu = Mu_hat * mu_mag_delGráfico

            mu = [x * mu_mag_delGráfico for x in Mu_hat]
            # Producto tensorial r_hat * r_hat^T
            AAA = (3 * np.outer(r_hat, r_hat)) - np.identity(3)
            # Primero multiplicamos AAA por Mu_hat, y luego por mu
            numerador = np.matmul(AAA, mu) 
            denominador = 4 * np.pi * (abs(length_r)**3)
            # Campo magnético B (vector)
            Campo_Magnetico_resultado = numerador / denominador  # a 40mm deberia marcar 400
            Campo_Magnetico_resultado = Campo_Magnetico_resultado * 1000000000000000
            
            return Campo_Magnetico_resultado



class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        self.RigidMO = kwargs['RigidMO']
        self.CFF = kwargs['CFF']
        self.CFFSphereROI = kwargs['CFFSphereROI']  
        self.t = 0
        print('Finished Init')

        # self.ModelNode = self.RootNode.solverNode.deformableNode.model  

    def MoveCFFSphereROI(self):   
        x = 10 * np.sin(self.t * 0.05) 
        y = -5 
        z = 3 
        self.CFFSphereROI.centers = [[x, y, z]]  
        self.t += 1  
        

    def onAnimateBeginEvent(self, eventType):
        
        self.MoveCFFSphereROI()
        self.CFF.totalForce.value = [0,0,-2000000]
        np.savetxt("MagnetPose_Direct.txt", self.RigidMO.position.value[1:, :])
        # print(f"MagnetPose: {self.RigidMO.position.value}")       
        
        
        
        MagnetPose = []
        MagnetPose = np.array(MagnetPose)
        try: 
            MagnetPose = np.loadtxt("MagnetPose_Direct.txt")
        except:
            print("error leyendo los datos desde archivo")
        # print("MagnetPose(Recibido desde Direct) ",MagnetPose)
        
        
        MagnetPosition = self.RigidMO.position.value[1:, :3]
        # print('Lista imanes:', MagnetPosition)
    
        if not hasattr(self, 'Lista_sensores'):
            self.SensorPosition = MagnetPosition.copy()
            self.SensorPosition[:, 2] -= Const.DeltaPositionSensor

       
        # print('Posicion sensores :', self.SensorPosition)
    
        SensorPosition = self.SensorPosition.copy()
        
        
        GlobalMagneticField = []

        for j in range(Const.NMagnets):
            LocalMagneticField = []
            Dist_Sensor = SensorPosition[j] - MagnetPosition
            # print(f'Distancia sensor {j} - imanes:', Dist_Sensor)
        
            for i in range(Const.NMagnets):
                
                # pos_iman = Lista_imanes[i]
                quat_iman = self.RigidMO.position.value[i+1, 3:7]
                # print(f"i : {i}, quat_iman{quat_iman} ")
                MiR = R.from_quat(quat_iman)  # (x, y, z, w)
    
                rotation_Matrix = MiR.as_matrix() 
                # print("rotation_Matrixxxxxxxxxxxxxxx: ", rotation_Matrix)
                Direccion_momento_magnetico = [rotation_Matrix[0, 2], rotation_Matrix[1, 2], rotation_Matrix[2, 2]]
                # print("Direccion momento magnetico: ", Direccion_momento_magnetico)
                # print("Calcularb: ",CalcularB(Dist_Sensor[i], Direccion_momento_magnetico, Const.mu_mag_delGráfico))
                LocalMagneticField.append(CalcularB(Dist_Sensor[i], Direccion_momento_magnetico, Const.mu_mag_delGráfico))
            # print("campo_local: ", len(campo_local)) 
            TotalMagneticField = np.sum(LocalMagneticField, axis=0)
            GlobalMagneticField.append(TotalMagneticField)
            # print(f"campototal sensor {j}", TotalMagneticField)
            
        np.savetxt("campo_global.txt", GlobalMagneticField)
        
        
        PointsSphereRoi = self.CFFSphereROI.pointsInROI.value

        
        # try: 
        CovM = np.cov(np.transpose(PointsSphereRoi))
        # print(CovM)
        EigVals, EigVecs = np.linalg.eig(CovM)
        minIdx = np.argmin(EigVals)
        directions = EigVecs[:,minIdx]

        # except:
            # directions = [0,0,-0]
            
        # print("DIRECTIOn", directions)
        print(f"PointsSphereRoi", PointsSphereRoi)
        
        
        
        # magnitude = -2e6  
        # force_vector = magnitude * directions / np.linalg.norm(directions)
        # self.CFF.totalForce.value = force_vector.tolist()
        # print(self.CFF.totalForce.value )
        
def createScene(rootNode):
    
    SofaRuntime.importPlugin("ArticulatedSystemPlugin")

    pluginsList = [
        'ArticulatedSystemPlugin',
        'SoftRobots',
        'SoftRobots.Inverse',
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
    scene.gravity = [0,  0, -9810]
    # scene.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')


    # ---- Paths to mesh files ----   
    VolumetricMeshPath = GeneratedMeshesPath + 'Sensor.vtk'
    
    SurfaceMeshPath = GeneratedMeshesPath + 'Sensor.stl'
        
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

    Boxes = []
    for i in range(len(Const.MagnetBoxCoords)):
        boxTip = completeMesh.addObject('BoxROI', name='Tip'+str(i), box=[Const.MagnetBoxCoords[i]], drawBoxes=True, 
                                        tetrahedra="@container.tetrahedra" , position="@container.position")
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
                        position=[[0., 0., Const.SensorHeight/2, 0., 0., 0., 1.]]) 
    servoBody.addObject('FixedProjectiveConstraint', indices=0)
    servoBody.addObject('UniformMass', totalMass=0.01)



    # ---- Articulation angle: 1 DOF rotation  ----
    articulationAngle = scene.Simulation.addChild('Articulation')
    articulationAngle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]],
                                rest_position=[[Const.ArticulationAngle]])
    articulationAngle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
    articulationAngle.addObject('UniformMass', totalMass=0.01)



    # ---- ServoWheel ----
    servoWheel = articulationAngle.addChild('ServoWheel')
    servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                         position=[[0., 0., Const.SensorHeight/2., 0., 0., 0., 1.], 
                                   [0., 0., Const.SensorHeight/2., 0., 0., 0., 1.]],
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
    print(nominal_pose)
    
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
    
    
    #Add the sensors 
    nominal_pose2 = [] 
    TipOrientation = [0, 0, 0, 1]
    for center in Const.SensorCenters:
        CurrentPose = center + TipOrientation
        nominal_pose2 += CurrentPose
    
    sensorCenter = servoWheel.addChild("sensorCenter")
    SensorMO = sensorCenter.addObject("MechanicalObject", name="dofs", template="Rigid3",
                         position=nominal_pose2,
                         showObject=True, showObjectScale=2)
    sensorCenter.addObject("UniformMass", totalMass=0.01)
    sensorCenter.addObject("EulerImplicitSolver")
    sensorCenter.addObject("SparseLDLSolver")
    sensorCenter.addObject("RigidMapping", input="@Simulation/Articulation/ServoWheel/dofs", rigidIndexPerPoint=Const.indexPerPointSensor)

    
    RigidifiedNode =  RigidNode.addChild('RigidifiedNode')   
    RigidifiedNode.addObject('MechanicalObject', name='RigidifiedMesh', position=pointsTip,
                             template='Vec3d', showObject=True, showObjectScale=4, showColor=1)       
    RigidifiedNode.addObject("RigidMapping", globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPoint)
    
    
    
    # ---- SubsetMultiMapping to connect ServoWheel and freeCenter to rigid tips ----
    RigidNode.addObject('SubsetMultiMapping',
                          name="mapping",
                          input=['@../dofs', '@/Simulation/freeCenter/dofs'],
                          output='@./', indexPairs= [Const.IndexPairs])
    
    
    
    # --- Define Articulation center ----
    articulationCenter = articulationAngle.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0], articulationIndex=0)
    articulationAngle.addObject('ArticulatedHierarchyContainer')
    
    
    
    # ---- Deformable Node -----
    deformableNode = RigidifiedNode.addChild("deformableNode")
    deformableNode.addObject('PointSetTopologyContainer', position=pointsBody)
    deformableNode.addObject('MechanicalObject', name='DeformableMech', showObject = False, showObjectScale = 4)
   
    
   
    model = deformableNode.addChild('model')
    # RigidifiedNode.addChild(model)
    
    
    
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
      
    
    
    
    
    
    # ##########################################
    # # Points On Surface                      #
    # ##########################################   
    # global ContactNodeMO, points 
    # points = np.load("Touch/PointsOnSurface.npy")
    # print("points",points)
    # # points_ordenados = sorted(points, key=lambda p: (float(p[0]), float(p[1])))
    # ContactNode = model.addChild("ContactNode")
    # ContactNodeMO = ContactNode.addObject("MechanicalObject", position=points, showColor=[0,0,200], showObjectScale=10, showObject=False,showIndices = True)
    # ContactNode.addObject("BarycentricMapping")



    # ----------------------------------------
    # Visualization                          
    # ----------------------------------------
    
    
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
    modelVisu.addObject('BarycentricMapping')


    
    	#---- Sphere ROI ---- 
    CFFNode = model.addChild('CFFNode')
    CFFNode.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    CFFMO = CFFNode.addObject('MechanicalObject', position='@loader.position') 
    CFFSphereROI = CFFNode.addObject('SphereROI', template="Vec3d", name='CFFSphereROI', centers=[[0,0,2]], radii=[1.25], drawSphere=True)
    CFFSphereROI.init()              
    CFF = CFFNode.addObject('ConstantForceField', name='CFF', template='Vec3', indices='@CFFSphereROI.indices', totalForce=[0, 0, 0])                               
    CFFNode.addObject("BarycentricMapping")
    

    rootNode.addObject(Controller(name="ActuationController", 
                                  RootNode=rootNode, 
                                  RigidMO=RigidMO,
                                  CFF=CFF,
                                  CFFSphereROI=CFFSphereROI)) 
    
    
    def animation(target, factor):
        target.dofs.rest_position[0][0] = np.sin(factor * 2 * np.pi)
        # print("Articulation angle:", target.dofs.position[0][0])
        # print("ServoWheel pose[1]:", servoWheel.dofs.position[1])
        print(nominal_pose)
    animate(animation, {'target': articulationAngle}, duration=2., mode='loop')
    
    return scene