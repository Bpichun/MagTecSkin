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


import rigidification
import Geometries.Constants as Const
datos_lista = []
global B_calculado_MagneticGoal

def CalcularB(Distancia_r_mm,Rotacion_Iman_Grados,mu_mag_delGráfico): #
            # Definición de variables
            Distancia_r = np.array(Distancia_r_mm)
            length_r = np.linalg.norm(Distancia_r) 
            # print("length_r", length_r)
            r_hat = Distancia_r / length_r
            # print("r_hat",r_hat)
            
            Mu_hat = Rotacion_Iman_Grados  # Dirección del momento magnético (vector unitario)

        # ······························································
            # Rotación (Definir rotación en CalcularB(x,Rotacion_Iman_Grados,x), no comentandolo)··········································
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
        self.MagnetEffector = kwargs['MagnetEffector']
        self.RigidMO = kwargs['RigidMO']
        # self.GoalDisplacement = np.array([Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ, 0,0,0,1])
        self.GoalDisplacement = np.array([0,0, Const.LargoViga , 0,0,0,1])
        self.InitialGoalPosition = self.GoalDisplacement
        
        print(kwargs['RootNode'])
        
        self.ModelNode = self.RootNode.solverNode.deformableNode.model        
        
        
        print('Finished Init')
        
    def onKeypressedEvent(self, c):
        key = c['key']        
            
        ##########################################
        # Cable                                  #
        ##########################################                
        
        Increment = 0.025
        if (key == "2"):
            self.GoalDisplacement = self.GoalDisplacement + [0,-Increment, 0, 0, 0, 0, 0]
            pass        
        if (key == "8"):
            self.GoalDisplacement = self.GoalDisplacement + [0,Increment, 0, 0, 0, 0, 0]
            pass            
        if (key == "4"):
            self.GoalDisplacement = self.GoalDisplacement + [-Increment, 0, 0, 0, 0, 0, 0]
            pass
        if (key == "6"):
            self.GoalDisplacement = self.GoalDisplacement + [Increment, 0, 0, 0, 0, 0, 0]
        if (key == "+"):
            self.GoalDisplacement = self.GoalDisplacement + [0, 0, Increment, 0, 0, 0, 0]
        if (key == "-"):
            self.GoalDisplacement = self.GoalDisplacement + [0, 0, -Increment, 0, 0, 0, 0]
        
            pass
        # self.MagnetEffector.effectorGoal.value = [self.GoalDisplacement] 
        
        #Diff = np.array(self.GoalDisplacement) - np.array(self.InitialGoalPosition)
        Diff = np.array(self.RigidMO.position.value) - np.array(self.RigidMO.rest_position.value)
        Orientation = R.from_quat(np.array(self.RigidMO.position.value)[0][3:])   
        # Orientation.as_euler
        # print(f'Effector Displacement: {Diff[0][0]:.3f},{Diff[0][1]:.3f},{Diff[0][2]:.3f}')
        # print(f'Effector Orientation (XYZ Euler, deg): {np.rad2deg(Orientation.as_euler("XYZ"))}')
        #print(f'EffectorOrientation Q: {Orientation[0]:.3f}, {Orientation[1]:.3f}, {Orientation[2]:.3f},{Orientation[3]:.3f}')
    
    

    def onAnimateBeginEvent(self, eventType):

        print("-------------------AnimateBeginEvent----------------------")
        
        MlxTxt = "../MlxInfo.txt"
        
        datos_lista = []
            	
        try: 
            datos_lista = np.loadtxt(MlxTxt)
        except:
            print("error leyendo los datos desde archivo")
        
        # print(datos_lista)
        datos_lista = [0,0,-150]
        datos_lista = np.append(datos_lista, [0, 0, 0,0])
        # print("Datos leídos del archivo:", datos_lista)

        Diff = np.array(self.RigidMO.position.value) - np.array(self.RigidMO.rest_position.value)
        # print(f"Diff: {Diff}")



        self.MagnetEffector.effectorGoal.value = [datos_lista]*-1 

        # Distancia_r_mm = [3.75 -5 2.7]
        Distancia_r_mm = [ 0.5,0,2.7]
        Rotacion_Iman_Grados = [0,0,1]
        mu_mag_delGráfico = 3.81e-9
        
        # B_calculado = CalcularB([0,0,2.7],Rotacion_Iman_Grados,mu_mag_delGráfico)
        # print("B_calculado de python inicial: ", B_calculado)

        # B_calculado = CalcularB(Distancia_r_mm,Rotacion_Iman_Grados,mu_mag_delGráfico)
        # print("B_calculado de python goal: ", B_calculado)
    


    #  V2:
        MagnetPose = []
        MagnetPose = np.array(MagnetPose)
        try: 
            MagnetPose = np.loadtxt("MagnetPose_Direct.txt")
        except:
            print("error leyendo los datos desde archivo")
        

        MiR = R.from_quat(MagnetPose[3:])  # (x, y, z, w)

        rotation_Matrix = MiR.as_matrix() 
        # print("rotation_Matrixxxxxxxxxxxxxxx: ", rotation_Matrix)
        Rotacion_Iman_Grados = [rotation_Matrix[0, 2], rotation_Matrix[1, 2], rotation_Matrix[2, 2]]
        # print("ROATACIOOOOOOOOOOOOOOOOOOOOOON: ", Rotacion_Iman_Grados)
        Distancia_r_mm = MagnetPose[:3]        
        B_calculado = CalcularB(Distancia_r_mm,Rotacion_Iman_Grados,mu_mag_delGráfico)
        # print("Tipo de B_calculado después de CalcularB:", type(B_calculado))
        # print("B_calculado con datos de Direct:", B_calculado)
        B_calculado = B_calculado.tolist()  # Convierte el ndarray en lista
        B_calculado += [0, 0, 0, 1]
        # print("B_calculado + 0 0 0 1:", B_calculado)
        # print("Tipo de B_calculado", type(B_calculado))
        B_calculado = [B_calculado]
        print("EffectorGoal Value: ", B_calculado)
        # self.MagnetEffector.effectorGoal.value = B_calculado







def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -0] #
                #rootNode.findData('gravity').value = [0, 0, -1000] #
                rootNode.findData('dt').value = 0.005

                rootNode.addObject('FreeMotionAnimationLoop')
                #rootNode.addObject('QPInverseProblemSolver', printLog='1', epsilon="1e-1", maxIterations="1000", tolerance="1e-5")
                rootNode.addObject('QPInverseProblemSolver', printLog=False, epsilon="0.0001", maxIterations="1000", tolerance="1e-5")

                #rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                #rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                #rootNode.addObject('CollisionPipeline', verbose="0")
                #rootNode.addObject('BruteForceDetection', name="N2")
                #rootNode.addObject('CollisionResponse', response="FrictionContact", responseParams="mu=0")
                #rootNode.addObject('LocalMinDistance', name="Proximity", alaefrmDistance="5", contactDistance="1")

                #rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.addObject('BackgroundSetting', color='0.85 0.85 0.85')
                
                
                rootNode.addObject('LightManager')
                rootNode.addObject(
                    "PositionalLight",
                    name="light1",
                    color="0.8 0.8 0.8",
                    # position=[0, Const.Thickness, 3],
                    position=[0, 0, -Const.SensorLength-20],

                )
                rootNode.addObject(
                    "PositionalLight",
                    name="light2",
                    color="0.8 0.8 0.8",
                    position=[0, 0, 0],
                )
                

                #VolumetricMeshPath = GeneratedMeshesPath + 'Cone_Volumetric.vtk'
                VolumetricMeshPath = GeneratedMeshesPath + '/Sensor.vtk'
                                
                      
                #SurfaceMeshPath = GeneratedMeshesPath + 'Cone_Surface.stl'
                SurfaceMeshPath = GeneratedMeshesPath + '/Sensor.stl'
                
                
                
                
                #----------------------
                # Goal Node
                #---------------------- 
                
             
                
                #----------------------
                # Rigidification - start
                #----------------------            
                
                completeMesh = rootNode.addChild('completeMesh')
                
                #completeMesh.addObject('RegularGrid',name='hexaGrid', nx="3", ny="3", nz="9", xmin="0", xmax="3", ymin="0", ymax="3", zmin="0", zmax="19")
                MeshTetra = completeMesh.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                completeMesh.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                completeMesh.init()
                # MeshTetra = completeMesh.addObject('Mesh', name="AllMesh", src='@loader')
                boxTip = completeMesh.addObject('BoxROI', name='Tip', box=Const.MagnetBoxCoords, drawBoxes=True, tetrahedra="@container.tetrahedra", position="@container.position")
                # boxTip  = completeMesh.addObject('SphereROI', template="Vec3d", name='SphereROI1', centers=[[Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ]], radii=[2], drawSphere=True)                            
                # boxTip  = completeMesh.addObject('SphereROI', template="Vec3d", name='SphereROI1', centers=[[0,0,24]], radii=[3], drawSphere=True) # Deberia dar 24, da 22
                # boxTip  = completeMesh.addObject('SphereROI', template="Vec3d", name='SphereROI1', centers=[[0,0,0]], radii=[3], drawSphere=True)
                # boxBody = completeMesh.addObject('BoxROI', name='MainBody', box=Const.MainBodyBoxCoords, drawBoxes=True, tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")
                #boxFixed = completeMesh.addObject('BoxROI', name='FixedPart', box=[-40, -40, -62, 40, 40, -46], drawBoxes='true', tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")   
                boxTip.init()
                # boxBody.init()
                #boxFixed.init()

                
                positionAllPoints = MeshTetra.findData('position').value;
                nbPoints = len(positionAllPoints)
                
                print(boxTip.indices.value)
                indicesTip= np.array(boxTip.indices.value);
                #indicesFixed= boxFixed.indices;
                #indicesTip = indicesFixed + indicesTip                
                print('indices tip' + str(indicesTip))
                indicesTip = indicesTip.flatten()
                print('indicesTip flatten ' + str(indicesTip))
                indicesTip = indicesTip.flatten()
                #indicesFixed = flatten(indicesFixed)
                
                #rigidBlocks = [indicesFixed+indicesTip]
                rigidBlocks = [indicesTip]            
                
                # indicesDeformable= np.array(boxBody.findData('indices').value);
                # indicesDeformable = indicesDeformable.flatten()
                
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
                print('indexPairs ')
                print(indexPairs)
                print('End of indexPairs ')
                # pointsBody = np.array(completeMesh.MainBody.pointsInROI.value).flatten().tolist()
                
                NPPointsDeformable = positionAllPoints[DeformableIndicesTotal,:]
                
                NPSortedPointsDeformable = NPPointsDeformable[IdxsOrderedFreeBlocks, :]
                PointsDeformable = NPSortedPointsDeformable.flatten().tolist()
                pointsBody = PointsDeformable
                #deformablePoints = pointsBody
                
                
                pointsTip = np.array(positionAllPoints[indicesTip,:]).flatten().tolist()                                                
                
                rigidIndexPerPoint = [0] * len(indicesTip)                
    
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
                for i in range(Const.NMagnets):
                # for i in range(1):
                    # CurrentPose = [Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ, 0,0,0,1]
                    CurrentPose = [Const.MagnetPosition[0], Const.MagnetPosition[1], Const.MagnetPosition[2], 0, 0, 0, 1]
                    nominal_pose += CurrentPose


                RigidMO = RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=nominal_pose, showObject=True, showObjectScale=2) # orientation is 240 deg away from scene origin
                # RigidMO = RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[0,0,24,0,0,0,1] , showObject=True, showObjectScale=2)
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

                model.addObject('MechanicalObject', name='tetras', template='Vec3',showObject = True,showObjectScale = 10 , showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
                model.addObject('UniformMass', totalMass='0.09')
                model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=Const.PoissonRatio,  youngModulus=Const.YoungsModulus)

                # model.addObject('BoxROI', name='boxROI', box=Const.BaseFixedBoxCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                  
                model.addObject('BoxROI', name='boxROI', box=Const.BoxROIFixCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")

                # print("self.RigidMO.rest_position.value", RigidMO.rest_position.value)
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

                #BLE = RigidNode.addObject('BendLabsEffector', template='Rigid3d', BaseCoord = [0,0,0]+BaseOrientation, idx=0, theta=0, phi=0, printLog=True)
                
                Goal = nominal_pose
                # Magneti Goal = [0,0,38.8*1000,0,0,0,1]
                MagneticGoal = [-7865.77954436  ,   0.     ,    27831.26441253,0,0,0,1] # -0.5,0,2.7 = 3.22985 -4.99945 2.69796

                # MagnetGoalPose = np.array([ 3.35643214, -4.91718177,  2.24329588,  0.08237773, -0.03229848, -0.04006265, 0.99527167])
                
                MagnetGoalPose = np.array([ 0,0,-24,  0, 0, 0, 1]) #Posición a la cual irá el imán
                R_est = R.from_quat([0.00057452 ,-0.0212411 ,-0.05052 ,0.998497]) # Obtenido de la simulación de SOFA
                R_tauth = R.from_quat(MagnetGoalPose[3:])                          # Posicion exacta a la que quiero llegar

                R_tauth_inv = R_tauth.inv()                
                R_Incognita = R_est * R_tauth_inv 
# 
                a = R.as_rotvec(R_Incognita)
                angulo_error = np.linalg.norm(a)

                print(f"angulo_error en orientación(deg)------------------------------------------: {np.rad2deg(angulo_error)}")

                RMagnet = R.from_quat(MagnetGoalPose[3:])
                RMagnetMatrix = RMagnet.as_matrix()
                RMagnetRotVec = RMagnet.as_rotvec()
                theta = np.linalg.norm(RMagnetRotVec)
                # print(f"RMagnetRotVec: {RMagnetRotVec}, dir: {RMagnetRotVec/theta}, theta (deg): {np.rad2deg(theta)}")
                mu_hat = RMagnetMatrix[:,2]

                BGoal = CalcularB(MagnetGoalPose[0:3]-[0,0,0],mu_hat,3.81e-9)

                print(f"BGoal: {BGoal}")
                # MagneticGoal = [7865.77954436  ,   0.     ,    27831.26441253,0,0,0,1] # 0.5,0,2.7 = 4.27279 -4.99994 2.69767

                # MagnetEffector = RigidNode.addObject('MagnetEffector', template='Rigid3d', indices=[0], useDirections=[0,0,1,0,0,0], effectorGoal = MagneticGoal, printLog=True)
                # MagnetEffector = RigidNode.addObject('MagnetEffector', template='Rigid3d',PosSensor = [0,0,-50,0,0,-24], indices=[0], useDirections=[0,0,1,0,0,0], effectorGoal = BGoal.tolist()+[0,0,0,1], printLog=True)
                # MagnetEffector = RigidNode.addObject('MagnetEffector', template='Rigid3d',PosSensor = [0,0,0,0,0,24], indices=[0], useDirections=[0,0,1,0,0,0], effectorGoal = [1,1,1]+[0,0,0,1], printLog=True)


                # MagnetEffector = RigidNode.addObject('MagnetEffector', template='Rigid3d',PosSensor = [0,0,-50,0,0,-24], indices=[0], useDirections=[0,0,1,0,0,0], effectorGoal = BGoal.tolist()+[0,0,0,1], printLog=True)



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
#                FPANode = RigidNode.addChild('FPANode')
#                FPAMO = FPANode.addObject('MechanicalObject', template='Rigid3', position=nominal_pose)
#                FPANode.addObject('ForcePointActuator', name='FPA1', template='Rigid3', direction='1 0 0 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False) #, showDirection=True, showVisuScale=10)                
#                FPANode.addObject('ForcePointActuator', name='FPA2', template='Rigid3', direction='0 1 0 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False)                
#                FPANode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False)                
#                #FPANode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100, minForce=-100, showForce=True, visuScale=20)                
#                FPANode.addObject("IdentityMapping")
#            
















    
                #FPA on deformable!
                FPANode = model.addChild('FPANode')
                FPANode.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
                
                FPAMO = FPANode.addObject('MechanicalObject', position='@loader.position') #position=[[1, -7.4, 2.3],[3.7, -7.4, 2.9],[6.4, -7.4, 2.3]], showObject=True, showObjectScale=20, showColor=[0,1,0])
                # FPASphereCenters = [[3.5, -6.5, 2.7],[3.7, -8.4, 2.6],[5.4, -8.4, 2.3]]
                FPASphereCenters = [[0,0,Const.SensorHeight],[0,0,0],[0,0,0]] # Ubicación de la fuerza
                diferencia = 1
                # FPASphereCenters = [[Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ-diferencia],[Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ-diferencia],[Const.SphereCenterX,Const.SphereCenterY,Const.SphereCenterZ-diferencia]]
                CenterIdx = 0
                FPASphereROI = FPANode.addObject('SphereROI', template="Vec3d", name='FPASphereROI', centers=[FPASphereCenters[CenterIdx]], radii=[1.25], drawSphere=True)
                # FPASphereROI.init()
                MaxVariation = 1000
#                 FPANode.addObject('ForcePointActuator', name='FPA1', template='Vec3', direction='1 0 0', indices='@FPASphereROI.indices', maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False, maxForceVariation=MaxVariation) #, showDirection=True, showVisuScale=10)                
#                 FPANode.addObject('ForcePointActuator', name='FPA2', template='Vec3', direction='0 1 0', indices='@FPASphereROI.indices', maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False, maxForceVariation=MaxVariation)                
#                 FPANode.addObject('ForcePointActuator', name='FPA3', template='Vec3', direction='0 0 1', indices='@FPASphereROI.indices', maxForce=100000, minForce=-100000, showForce=True, visuScale=1, printLog=False, maxForceVariation=MaxVariation)                
# #                #FPANode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100, minForce=-100, showForce=True, visuScale=20)                
                FPANode.addObject("BarycentricMapping")
                
                MarkerNode = model.addChild('MarkerNode')
                MarkerMO = MarkerNode.addObject('MechanicalObject', position=[FPASphereCenters[CenterIdx]], showObject=True, showObjectScale=20, showColor=[0,1,0])
                MarkerNode.addObject('BarycentricMapping')
#                
#                # Use force point actuators somewhere along the structure
#                FPANode = model.addChild('FPANode')
#                FPANode.addObject('MechanicalObject', template='Vec3d', position=[[0, 0, 60]])
#                FPANode.addObject('ForcePointActuator', name='FPA1', template='Vec3d', direction='1 0 0', indices=[0], maxForce=30, minForce=-30, maxForceVariation=0.8, showForce=True, visuScale=5) #, showDirection=True, showVisuScale=10)                
#                FPANode.addObject('ForcePointActuator', name='FPA2', template='Vec3d', direction='0 1 0', indices=[0], maxForce=30, minForce=-30, maxForceVariation=0.8, showForce=True, visuScale=5)                
#                FPANode.addObject("BarycentricMapping")
#                
#                
                # GoalNode = rootNode.addChild('GoalNode')
                # GoalMO = GoalNode.addObject("MechanicalObject", template="Rigid3d", position=[0,0,Const.Height,0,0,0,1], showObject=True, showObjectScale=10)
                
                
                # rootNode.addObject(OrientationSweepController(name="OrientationSweeController", BendLabsEffector=BLE, FPAMO=FPAMO,GoalMO=GoalMO))
                #RigidNode.addObject('ForcePointActuator', name="FPA1", indices=[0], showForce=1, visuScale=0.1)
                #FPAs.addObject('ForcePointActuator', name="FPA2", indices=[0], direction=[0,1,0], showForce=1, visuScale=0.1)
                #FPAs.addObject('ForcePointActuator', name="FPA3", indices=[0], direction=[1,0,0], showForce=1, visuScale=0.1)                
                #FPAs.addObject('ForcePointActuator', name="FPA2", indices=[], direction=[0,0,1], showForce=1, visuScale=0.1)                
                #FPAs.addObject('BarycentricMapping', mapForces="true", mapMasses="false")

                # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
                # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
                # to the finger and vice-versa;
                
                
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, MagnetEffector=MagnetEffector, RigidMO=RigidMO))    
                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, MagnetEffector=MagnetEffector, ContactNodeMO = ContactNodeMO,FPASphereROI = FPASphereROI, PA = FPA, RigidMO=RigidMO))    





                return rootNode
