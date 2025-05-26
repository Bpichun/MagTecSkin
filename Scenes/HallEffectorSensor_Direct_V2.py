#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 19 09:29:11 2025

@author: benjamin
"""

import Sofa
import os
import numpy as np
from scipy.spatial.transform import Rotation as R 
#from stlib.physics.deformable import ElasticMaterialObject
#from stlib.physics.constraints import FixedBox
#from stlib.physics.collision import CollisionMesh


#from stlib.visuals import VisualModel

#from softrobots.actuators import PneumaticCavity
#from softrobots.actuators import VolumeEffector
#from softrobots.sensors  import PneumaticSensor
path = os.path.dirname(os.path.abspath(__file__))+'/Meshes/'
MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

TempPath = os.path.dirname(os.path.abspath(__file__))+'/Temp/'

import Geometries.Constants as Const
import rigidification
Lista_sensores = []

def CalcularB(Distancia_r_mm,Direccion_momento_magnetico,mu_mag_delGráfico): #
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
        #self.PositionEffector = kwargs['PositionEffector']
        self.RigidMO = kwargs['RigidMO']
        self.CFF = kwargs['CFF']
        self.CFFSphereROI = kwargs['CFFSphereROI']
        self.ForceVector = np.array([0,0,0])
        self.InitialGoalPosition = self.ForceVector


        print(kwargs['RootNode'])
        
        self.ModelNode = self.RootNode.solverNode.deformableNode.model        
        self.t = 0
        
        
        
        self.t = 0

        self.cx = 2
        self.cy = 2
        self.cz = 0.1
        print('Finished Init')
        

        
    def onKeypressedEvent(self, c):
        key = c['key']        
        
        print('blup')
        ##########################################
        # Cable                                  #
        ##########################################                
        
        # Increment = 50000
        # if (key == "a"):
        #     self.ForceVector = self.ForceVector + [0,-Increment, 0]
            
        # if (key == "8"):
        #     self.ForceVector = self.ForceVector + [0,Increment, 0]
        # if (key == "4"):
        #     self.ForceVector = self.ForceVector + [-Increment,0, 0]            
        # if (key == "6"):
        #     self.ForceVector = self.ForceVector + [Increment,0, 0]
        # if (key == "+"):
        #     self.ForceVector = self.ForceVector + [0, 0, Increment]
        # if (key == "-"):
        #     self.ForceVector = self.ForceVector + [0, 0, -Increment]
        
                            
        # self.CFF.totalForce.value = self.ForceVector.tolist()                       
        # #Diff = np.array(self.ForceVector) - np.array(self.InitialGoalPosition)     
                
        # print(f"MagnetPose: {self.RigidMO.position.value}")
        # Diff = np.array(self.RigidMO.position.value) - np.array(self.RigidMO.rest_position.value)
        # Orientation = R.from_quat(np.array(self.RigidMO.position.value)[0][3:])   
        
        
        # # Orientation.as_euler
        # print(f'Effector Displacement: {Diff[0][0]:.3f},{Diff[0][1]:.3f},{Diff[0][2]:.3f}')
        # print(f'Effector Orientation (XYZ Euler, deg): {np.rad2deg(Orientation.as_euler("XYZ"))}')
        # #print(f'EffectorOrientation Q: {Orientation[0]:.3f}, {Orientation[1]:.3f}, {Orientation[2]:.3f},{Orientation[3]:.3f}')
        
        
        sphere_increment = 0.1
        x, y, z = self.CFFSphereROI.centers[0]

        if key == "1":
            y -= sphere_increment
        if key == "2":
            y += sphere_increment
        if key == "3":
            x -= sphere_increment
        if key == "5":
            x += sphere_increment
        if key == "i":
            z -= sphere_increment
        if key == "o":
            z += sphere_increment
    
        self.CFFSphereROI.centers = [[x, y, z]]
        self.CFF.totalForce.value = [0,0,-20000 ]
        print(f"Esfera ROI nueva posición: {[x, y, z]}")
        
        
        
    def MoveCFFSphereROI(self):   
        x = 12 * np.sin(self.t * 0.03) 
        y = 5 * np.sin(self.t * 0.03) 
        z = 3
        
        
        cx = 2 
        cy = 2 
        cz = 0.1 
        
        self.CFFSphereROI.centers = [[x, y, z]]  
        self.t += 1  
   
    
    # def MoveCFFSphereROI(self):
    #     # Parámetros del rectángulo
    #     width = 10
    #     height = 6
    #     z = 3
    
    #     # Control de velocidad
    #     velocidad = 10  
    #     step = (self.t // velocidad) % (2 * (width + height))
    
    #     if step < width:
    #         x = step
    #         y = -height
    #     elif step < width + height:
    #         x = width
    #         y = step - width - height
    #     elif step < 2 * width + height:
    #         x = width - (step - (width + height))
    #         y = height
    #     else:
    #         x = -width
    #         y = height - (step - (2 * width + height))
    
    #     self.CFFSphereROI.centers = [[x, y, z]]
    #     self.t += 1

    
   
    
    def MoveCFFBoxROI(self):

        x = 10 * np.sin(self.t * 0.05)
        y = -5
        z = 3

        self.CFFSphereROI.box = [
            x - self.cx, y - self.cy, z - self.cz,
            x + self.cx, y + self.cy, z + self.cz
        ]

        self.t += 1
    
   
    def onAnimateBeginEvent(self, eventType):
        # global Lista_sensores
        self.MoveCFFSphereROI()
        
        
        
        # self.MoveCFFBoxROI()
        # # self.onKeypressedEvent()
        # # self.mapCapCoordinatesTo3DCoords()
        self.CFF.totalForce.value = [0,0,-20000]
        # np.savetxt("MagnetPose_Direct.txt", self.RigidMO.position.value)
        # print(f"MagnetPose: {self.RigidMO.position.value}")       
        
        
        MagnetPose = []
        MagnetPose = np.array(MagnetPose)
        try: 
            MagnetPose = np.loadtxt("MagnetPose_Direct.txt")
        except:
            print("error leyendo los datos desde archivo")


        print("MagnetPose(Recibido desde Direct) ",MagnetPose)
        
        
        

        MagnetPosition = self.RigidMO.position.value[:, :3]
        print('Lista imanes:', MagnetPosition)
    
        if not hasattr(self, 'Lista_sensores'):
            self.SensorPosition = MagnetPosition.copy()
            self.SensorPosition[:, 2] -= Const.DeltaPositionSensor

       
        print('Posicion sensores :', self.SensorPosition)
    
        SensorPosition = self.SensorPosition.copy()
        
        
        GlobalMagneticField = []

        for j in range(Const.NMagnets):
            LocalMagneticField = []
            Dist_Sensor = SensorPosition[j] - MagnetPosition
            # print(f'Distancia sensor {j} - imanes:', Dist_Sensor)
        
            for i in range(Const.NMagnets):
                
                # pos_iman = Lista_imanes[i]
                quat_iman = self.RigidMO.position.value[i, 3:7]
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
            print(f"campototal sensor {j}", TotalMagneticField)
            
        np.savetxt("campo_global.txt", GlobalMagneticField)
        
        print("---------------------------------------------------------------")

        
        # for r in len(campo_local):
            
        
            # Distancia = Lista_sensores[i] - Lista_imanes
            # for dist in Distancia:
            #     Distancia_r_mm = dist
            #     campoMagnetico = CalcularB(Distancia_r_mm, Direccion_momento_magnetico, Const.mu_mag_delGráfico)
            #     campo_local.append(campoMagnetico)
            #     suma = np.sum(campo_local)
            # campo_global.append(suma)    
                
                
                
        # delta_Z = 1
        # Lista_imanes = self.RigidMO.position.value
        # Lista_imanes= Lista_imanes[:, :3] 
        # print('Lista imanes', Lista_imanes)
        # print('Lista sensores 1 ', Lista_sensores)

        # Lista_sensores_ = Lista_sensores.copy()

        # Lista_sensores_[:, 2] -= delta_Z
        # print('Lista sensores', Lista_sensores)

        # Dist_Sensor1 =  Lista_sensores_[0] - Lista_imanes
        # # Diff_Sensor_Iman = Lista_imanes - Lista_sensores
        # #DistSensor1 se entrega a calculoB, y se suman los n campos magneticos
        

        
def createScene(rootNode):
                
                # global Lista_sensores
                
                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -9810] #
                #rootNode.findData('gravity').value = [0, 0, -1000] #
                rootNode.findData('dt').value = 0.02

                rootNode.addObject('FreeMotionAnimationLoop')
                #rootNode.addObject('QPInverseProblemSolver', printLog='1', epsilon="1e-1", maxIterations="1000", tolerance="1e-5")
                #rootNode.addObject('QPInverseProblemSolver', printLog=False, epsilon="0.0001", maxIterations="1000", tolerance="1e-5")

                rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                #rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.addObject('BackgroundSetting', color='0.85 0.85 0.85')
                
                
                rootNode.addObject('LightManager')
                rootNode.addObject(
                    "PositionalLight",
                    name="light1",
                    color="0.8 0.8 0.8",
                    position=[0, 0, 10],
                )
                rootNode.addObject(
                    "PositionalLight",
                    name="light2",
                    color="0.8 0.8 0.8",
                    position=[0, 0, -10],
                )
                

                #VolumetricMeshPath = GeneratedMeshesPath + 'Cone_Volumetric.vtk'
                VolumetricMeshPath = GeneratedMeshesPath + 'Sensor.vtk'
                                
                      
                #SurfaceMeshPath = GeneratedMeshesPath + 'Cone_Surface.stl'
                SurfaceMeshPath = GeneratedMeshesPath + 'Sensor.stl'
                
                
                
                
                #----------------------
                # Goal Node
                #---------------------- 
                
             
                
                #----------------------
                # Rigidification - start
                #----------------------          
                
                
                
                
                
                completeMesh = rootNode.addChild('completeMesh')
                
                #completeMesh.addObject('RegularGrid',name='hexaGrid', nx="3", ny="3", nz="9", xmin="0", xmax="3", ymin="0", ymax="3", zmin="0", zmax="19")
                completeMesh.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                completeMesh.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                completeMesh.init()
                MeshTetra = completeMesh.addObject('MeshTopology', name="AllMesh", src='@loader')
               
                
               
  
                
               
               
                # boxTip = completeMesh.addObject('BoxROI', name='Tip', box=Const.MagnetBoxCoords, drawBoxes=True,tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")
                # boxTip = completeMesh.addObject('BoxROI', name='Tip', box=Const.MagnetBoxCoords, drawBoxes=True,tetrahedra="@container.tetrahedra" , position="@container.position")
                # boxTip = completeMesh.addObject('BoxROI', name='Tip', box=Const.MagnetBoxCoords[1], drawBoxes=True,tetrahedra="@container.tetrahedra" , position="@container.position")
                # boxTip = completeMesh.addObject('SphereROI', template="Vec3d", name='SphereROI1', centers=[[0,0,3]], radii=[1], drawSphere=True)

                # boxTip  = completeMesh.addObject('SphereROI', template="Vec3d", name='SphereROI1', centers=[[0,0,Const.SensorHeight]], radii=[3], drawSphere=True)                

                # boxBody = completeMesh.addObject('BoxROI', name='MainBody', box=Const.MainBodyBoxCoords, drawBoxes=True, tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")
                #boxFixed = completeMesh.addObject('BoxROI', name='FixedPart', box=[-40, -40, -62, 40, 40, -46], drawBoxes='true', tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")   
                # boxTip.init()
                # boxBody.init()
                #boxFixed.init()
                Boxes = []
                for i in range(0, Const.NMagnets):
                    boxTip = completeMesh.addObject('BoxROI', name='Tip'+str(i), box=[Const.MagnetBoxCoords[i]], drawBoxes=True, tetrahedra="@container.tetrahedra" , position="@container.position")
                    Boxes.append(boxTip)
                    boxTip.init()
                
                # boxTip = Boxes[-1]
                
                
                
                positionAllPoints = MeshTetra.findData('position').value;
                nbPoints = len(positionAllPoints)
                
                
                IndicesWithRigidIdx = np.empty((0,2), dtype=int)
                
                for (i,Box) in enumerate(Boxes):
                    IndicesNP = np.array(Box.indices.value, dtype=int)
                    NPoints = len(IndicesNP)
                    RigidIdx = np.ones(NPoints,dtype=int)*i
                    CurrentIndicesWithRigidIdx = np.append(IndicesNP.reshape((NPoints,1)), RigidIdx.reshape((NPoints,1)),1)
                    IndicesWithRigidIdx = np.append(IndicesWithRigidIdx, CurrentIndicesWithRigidIdx,0)
                
                # print(f"IndicesWithRigidIdx: {IndicesWithRigidIdx}")
                IndicesWithRigidIdxSorted = np.sort(IndicesWithRigidIdx[:,0],0)
                SortedIdxs = np.argsort(IndicesWithRigidIdx[:,0],0)
                SortedRigidIdxs = IndicesWithRigidIdx[:,1][SortedIdxs]
                # print(f"SortedRigidIdxs: {SortedRigidIdxs}")
                
                
                indicesTip = IndicesWithRigidIdxSorted.tolist()
                
                rigidBlocks = [IndicesWithRigidIdxSorted.tolist()] 
                
                DeformableIndicesTotal = []    
                
                
                
                # print(boxTip.indices.value)
                # indicesTip= np.array(boxTip.indices.value);
                # #indicesFixed= boxFixed.indices;
                # #indicesTip = indicesFixed + indicesTip                
                # print('indices tip' + str(indicesTip))
                # indicesTip = indicesTip.flatten()
                # print('indicesTip flatten ' + str(indicesTip))
                # indicesTip = indicesTip.flatten()
                # #indicesFixed = flatten(indicesFixed)
                
                # #rigidBlocks = [indicesFixed+indicesTip]
                # rigidBlocks = [indicesTip]            
                
                # # indicesDeformable= np.array(boxBody.findData('indices').value);
                # # indicesDeformable = indicesDeformable.flatten()
                
                DeformableIndicesTotal = []    

                for i in range(nbPoints):
                    if i not in indicesTip:
                        DeformableIndicesTotal.append(i)                                 

                freeBlocks = np.sort(DeformableIndicesTotal)    
                IdxsOrderedFreeBlocks = np.argsort(DeformableIndicesTotal)    
                
                
                
                # freeBlocks = indicesDeformable
                print(nbPoints)
                print('+++++++++++++++++++++++++++++++++++')
                print(rigidBlocks)
                print('+++++++++++++++++++++++++++++++++++')
                print(freeBlocks)
                print ('+++++++++++++++++++++++++++++++++++')
#
                indexPairs = np.array(rigidification.fillIndexPairs(nbPoints,freeBlocks,rigidBlocks))
                # print('indexPairs ')
                # print(indexPairs)
                # print('End of indexPairs ')
                # pointsBody = np.array(completeMesh.MainBody.pointsInROI.value).flatten().tolist()
                
                NPPointsDeformable = positionAllPoints[DeformableIndicesTotal,:]
                
                NPSortedPointsDeformable = NPPointsDeformable[IdxsOrderedFreeBlocks, :]
                PointsDeformable = NPSortedPointsDeformable.flatten().tolist()
                pointsBody = PointsDeformable
                #deformablePoints = pointsBody
                
                
                pointsTip = np.array(positionAllPoints[indicesTip,:]).flatten().tolist()                                                
                
                
                rigidIndexPerPoint = SortedRigidIdxs.tolist()
                # rigidIndexPerPoint = [0] * len(indicesTip)                
    
                solverNode = rootNode.addChild("solverNode")
                solverNode.addObject('EulerImplicitSolver',rayleighStiffness="0.1", rayleighMass="0.1")
                solverNode.addObject('SparseLDLSolver',name='preconditioner')
                solverNode.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                # solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model' )
                #solverNode.addObject('MechanicalMatrixMapper', template='Rigid,Rigid', object1='@./RigidNode/RigidMesh', object2='@./RigidNode/RigidMesh', nodeToParse='@./RigidNode/RigidifiedNode', stopAtNodeToParse=True )
                deformableNode = solverNode.addChild("deformableNode")
                deformableNode.addObject('PointSetTopologyContainer', position=pointsBody)
                deformableNode.addObject('MechanicalObject', name='DeformableMech')
                #deformableNode.addObject('BoxROI', name='BoxForSliding', box=Constants.SlidingBoxCoords, drawBoxes='true')
                #deformableNode.addObject('PartialFixedConstraint', indices='@BoxForSliding.indices', fixedDirections=[0,0,1])                
                
                
                RigidNode= solverNode.addChild('RigidNode')

                #RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[[0, 0, Const.Height, 0.23622992, 0.30639972, 0.12644464, 0.91341469]], showObject=True, showObjectScale=5) # orientation is 240 deg away from scene origin
                
                #TipOrientation = [0.14224426, 0.0996005 , 0.56486252, 0.80670728]#  #[0.       , 0.       , 0.5      , 0.8660254] #[0, 0., 0.70710678, -0.70710678] #
                
                #TipOrientation = [1,0,0,0]
                
                TipOrientation = [0, 0, 0, 1]    
               
                nominal_pose = [] 
                for center in Const.MagnetCenters:
                    # CurrentPose = [Const.MagnetCenters[i],Const.MagnetCenters[i],Const.MagnetCenters[i], 0,0,0,1]
                    CurrentPose = center + [0,0,0,1]
                    nominal_pose += CurrentPose
                RigidMO = RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=nominal_pose, showObject=True, showObjectScale=1,showIndices=True) # orientation is 240 deg away from scene origin
                
                # Lista_sensores = RigidMO.position.value
                
                #RigidNode.addObject('BoxROI', name='BoxForSliding', box=Constants.SlidingBoxCoords, drawBoxes='true')
                #RigidNode.addObject('PartialFixedConstraint', indices=[0], fixedDirections=[0,0,0,1,1,0])
             
                RigidifiedNode= RigidNode.addChild('RigidifiedNode')
                
                RigidifiedNode.addObject('MechanicalObject', name='RigidifiedMesh', position = pointsTip, template = 'Vec3d', showObject = True, showObjectScale = 10, showColor = 1)
                
                RigidifiedNode.addObject("RigidMapping",globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPoint)  
                
                model = deformableNode.addChild('model')
                RigidifiedNode.addChild(model)
                
                	#Heart
                model.addObject('EulerImplicitSolver', name='odesolver')
                #model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond=True, update_step='1')
                model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', update_step='1')
    
                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')

                model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
                model.addObject('UniformMass', totalMass='0.09')
                model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=Const.PoissonRatio,  youngModulus=Const.YoungsModulus)

                # model.addObject('BoxROI', name='boxROI', box=Const.BaseFixedBoxCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                
                model.addObject('BoxROI', name='boxROI', box=Const.BoxROIFixCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                
                
                
                model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')

#               model.addObject('SparseLDLSolver', name='preconditioner')
#                model.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')
#    
#                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
#                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
#                
#                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
#                model.addObject('UniformMass', totalMass='0.3')
#                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Constants.PoissonRation,  youngModulus=Constants.YoungsModulus, drawAsEdges="0")   
#                   
                model.addObject("SubsetMultiMapping",name="subsetMapping",template="Vec3d,Vec3d", input='@'+deformableNode.getPathName()+'/DeformableMech' + ' ' + '@'+RigidifiedNode.getPathName()+'/RigidifiedMesh' , output='@./tetras', indexPairs=indexPairs.tolist())
          
                
#                #----------------------
#                # Rigidification - end
#                #----------------------
                   
#                model = rootNode.addChild('model')
#                model.addObject('EulerImplicit', name='odesolver')
#                model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
#                model.addObject('SparseLDLSolver', name='precond')
#
#                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
#                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
#                model.addObject('TetrahedronSetTopologyModifier')
#
#                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
#                model.addObject('UniformMass', totalMass='0.1')
#                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Const.PoissonRatio,  youngModulus=Const.YoungsModulus)
#
#                #model.addObject('BoxROI', name='boxROI', box='-10 -15 50 10 15 90', drawBoxes='true')
#                #model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness="1e2")
#                
#                model.addObject('BoxROI', name='BoxROI1', box=Const.BaseFixedBoxCoords, drawBoxes=True)
#                model.addObject('RestShapeSpringsForceField', points='@BoxROI1.indices', stiffness='1e12')        
#
#                model.addObject('LinearSolverConstraintCorrection', name='GCS', solverName='precond')



		        ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = model.addChild('visu')
                modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
                modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
                modelVisu.addObject('BarycentricMapping')
                
                ##########################################
                # Effector                               #
                ########################################## 

                #MOPositions = [[0,0,0,0,0,1,0], [0,0,-100,0,0,1,0]]
                #Sensor = RigidNode.addChild('BendLabsSensor')
                
                #RigidNode.addObject('MechanicalObject', name='SensorPoints', template="Rigid3d", showObject=True, showObjectScale=5, position=MOPositions)
                
                #goal = rootNode.addChild('goal')
                #goal.addObject('MechanicalObject', name="goalMO", template='Rigid3d', position=[[0,10,-90,0., 0. , 0.25881905, -0.96592583]], showObject=True, showObjectScale=10)
                #RigidNode.addObject('PositionEffector', template='Rigid3d', indices=[0], effectorGoal="@../../goal/goalMO.position", useDirections='0 1 0 0 0 1')
                
                
                #effector.addObject('PositionEffector', indices=[i for i in range(len(goalMO.position.value))], template='Vec3', effectorGoal="@../../goal/goalMO.position", useDirections='1 1 1')
              
                #RigidNode.addObject('BendLabsEffector', template='Rigid3d', alpha=0.39269908169872414, beta=0.39269908169872414)
               
                
                BaseOrientation = TipOrientation
                #BaseOrientation = [1,0,0,0]
                #BaseOrientation = [0.        , 0.        , 0.70710678, 0.70710678]
                #BaseOrientation = [0.        , 0.        , 0.38268343, 0.92387953]
                #BLE = RigidNode.addObject('BendLabsEffector', template='Rigid3d', BaseCoord = [0,0,0]+BaseOrientation, idx=0, theta=0, phi=0, printLog=True)
                
                Goal = nominal_pose
                
                #PositionEffector = RigidNode.addObject('PositionEffector', template='Rigid3d', indices=[0], useDirections=[1,1,1,0,0,0], effectorGoal = Goal, printLog=True)
                #RigidNode.addObject('BarycentricMapping')
                
                
#                MOPositions = [[0,0,0], [0,0,-100]]
#                RigidFrame = model.addChild('RigidFrame')
#                RigidFrame.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[0, 0,-100, 0, 0, 1, 0], showObject=True, showObjectScale=20) # orientation is 240 deg away from scene origin
#               
#                RigidFrame.addObject('BarycentricMapping')


#                ##########################################
#                # Actuation                              #
#                ########################################## 
#                 
#                FPAs = model.addChild('ForcePointActuators')           
#                Force1ApplicationPoint = [[0,0,-100]] 
#                FPAs.addObject('MechanicalObject', name="FPAsMOs", showObject='True', showObjectScale='3', position=Force1ApplicationPoint)        
                
                # Use sliding actuators
#                SANode = RigidNode.addChild('SANode')
#                SANode.addObject('MechanicalObject', template='Rigid3', position=[0,0,Const.Height,0,0,1,0])
#                SANode.addObject('SlidingActuator', name='SA1', template='Rigid3', direction='1 0 0 0 0 0', indices=0, maxForce=100, minForce=-100) #, showDirection=True, showVisuScale=10)                
#                SANode.addObject('SlidingActuator', name='SA2', template='Rigid3', direction='0 1 0 0 0 0', indices=0, maxForce=100, minForce=-100)
#                SANode.addObject("IdentityMapping")
                
                
                
                
                
#                #FPA on rigid!
#                CFFNode = RigidNode.addChild('CFFNode')
#                FPAMO = CFFNode.addObject('MechanicalObject', template='Rigid3', position=nominal_pose)
#                CFFNode.addObject('ForcePointActuator', name='FPA1', template='Rigid3', direction='1 0 0 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False) #, showDirection=True, showVisuScale=10)                
#                CFFNode.addObject('ForcePointActuator', name='FPA2', template='Rigid3', direction='0 1 0 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False)                
#                CFFNode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False)                
                # CFFNode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100, minForce=-100, showForce=True, visuScale=20)                
#                CFFNode.addObject("IdentityMapping")
# 
               
                #FPA on deformable!
                CFFNode = model.addChild('CFFNode')
                CFFNode.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
                # CFFNode.addObject('MeshSTLLoader', filename="Geometries/disk.stl", name="diskLoader")

                
                CFFMO = CFFNode.addObject('MechanicalObject', position='@loader.position') #position=[[1, -7.4, 2.3],[3.7, -7.4, 2.9],[6.4, -7.4, 2.3]], showObject=True, showObjectScale=20, showColor=[0,1,0])
                # # CFFSphereCenters = [[1, -8.4, 2.1],[3.7, -8.4, 2.6],[5.4, -8.4, 2.3]]
                # CFFSphereCenters = [[3.5, -6.5, 2.7],[3.7, -8.4, 2.6],[5.4, -8.4, 2.3]]
                # CenterIdx = 0
                CFFSphereROI = CFFNode.addObject('SphereROI', template="Vec3d", name='CFFSphereROI', centers=[[0,0,3]], radii=[2], drawSphere=False)
                # CFFNode.addObject('MeshROI', name="ROIm", drawBox="0", drawEdges="0", drawTriangles="1", drawTetrahedra="1", drawOut="0", computeMeshROI="1", doUpdate="0", position="@../mecaObj.position", tetrahedra="@../loader.tetrahedra", ROIposition="@ROIloader.position", ROItriangles="@ROIloader.triangles")
                # CFFSphereROI = CFFNode.addObject('BoxROI',name='CFFCircleROI11', box=[-2, -2, 2.95, 2, 2, 3.05], drawBoxes=True)
                
                # boxTip = completeMesh.addObject('BoxROI', name='Tip'+str(i), box=[Const.MagnetBoxCoords[i]], drawBoxes=True, tetrahedra="@container.tetrahedra" , position="@container.position")




                
                
                
                CFFSphereROI.init()                
                CFF = CFFNode.addObject('ConstantForceField', name='CFF1', template='Vec3', indices='@CFFSphereROI.indices', totalForce=[0, 0, 0]) #, showDirection=True, showVisuScale=10)                               
                CFFNode.addObject("BarycentricMapping")
                
                MarkerNode = model.addChild('MarkerNode')
                MarkerMO = MarkerNode.addObject('MechanicalObject', position=[[0,0,0]], showObject=True, showObjectScale=20, showColor=[0,1,0])
                MarkerNode.addObject('BarycentricMapping')
   
                
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, RigidMO=RigidMO, CFF=CFF, CFFSphereROI=CFFSphereROI))  

                # rootNode.addObject(Controller(name="ActuationController",
                #                             RootNode=rootNode,
                #                             ContactNode=ContactNode, 
                #                             RigidMO=RigidMO,
                #                             CFF=CFF,
                #                             ContactNodeMO = ContactNodeMO,
                #                             CFFSphereROI = CFFSphereROI,
                #                             DeformableMech = DeformableMech))                    
                
                


                return rootNode