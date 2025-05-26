#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Created on Tue Mar 12 16:53:39 2024

@author: stefan
"""

import gmsh
import numpy as np
# import TrianglePipe
# import Constants 


gmsh.initialize()
factory = gmsh.model.occ
model = gmsh.model
launchGUI = gmsh.fltk.run


# FileName = 'Scenes/Geometries/Dome_V2_StepAbril.step'
FileName = "Dome_DomoPequeño.step"



def rotate_model(angle):
    factory.rotate([DimTag], 0, 0, 0, 0, 0, 1, angle)

def evaluate_coordinate(count, HorizontalCoordinates):
    # if count == 0:  HorizontalCoordinates = HorizontalCoordinates[nx//2-1:nx//2+3]
    # elif count == 1:  HorizontalCoordinates = HorizontalCoordinates[nx//2-1:nx//2+3]
    # elif count == 2:  HorizontalCoordinates = HorizontalCoordinates[nx//2-6:nx//2+7]
    # elif count == 3:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+7]
    # elif count == 4:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+7]
    # elif count == 5:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+7]
    # elif count == 6:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+6]
    # elif count == 7:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+6]
    # elif count == 8:  HorizontalCoordinates = HorizontalCoordinates[nx//2-5:nx//2+5]
    # elif count == 9: HorizontalCoordinates = HorizontalCoordinates[nx//2-4:nx//2+4]
    return HorizontalCoordinates

def calcCenterOfMassObject(DimTag, z_offset=-10.5): 
    '''
        Calculate the center of mass of the object
    '''
    BoundaryPoints = model.getBoundary([DimTag], recursive=True)  # Get boundary points for the object
    CoordinatesBoundaryPoints = np.empty((0, 3))  # Initialize empty array for storing coordinates
    print(f'BoundaryPoints: {BoundaryPoints}')  
    for PointTag in BoundaryPoints:  # Loop through each boundary point
        print(f'PointTag: {PointTag}') 
        Coordinates = model.getValue(0, PointTag[1], [])  # Get coordinates
        CoordinatesBoundaryPoints = np.append(CoordinatesBoundaryPoints, [Coordinates], 0) 
    center_of_mass = np.mean(CoordinatesBoundaryPoints, 0)  # Calculate mean coordinates
    center_of_mass[2] += z_offset  # Adjust z coordinate
    return center_of_mass 


def getCoordinates(CenterOfMass, NX=6, NY=6, XWidth=50, YHeight=50, XOffset=0, YOffset=0):
    '''
        Get coordinates for the cutting planes  (only for the X and Y axis) (lenght is not included)
    '''
    global nx, ny
    nx = NX + 4
    ny = NY + 4
    XIncrement = XWidth / (NX - 1) 
    YIncrement = YHeight / (NY - 1)

    PlanesDimTags = []
    HorizontalCoordinates = []
    for i in range(NX):
        # adding horizontal coordinates
        HorizontalCoordinates.append(CenterOfMass[0] - XWidth / 2 + i * XIncrement + XOffset)
    VerticalCoordinates = []
    for i in range(NY):
        # adding vertical coordinates
        VerticalCoordinates.append(CenterOfMass[1] - YHeight / 2 + i * YIncrement + YOffset)

    FrameDensityFactor = 2/NX
    FrameSize =  6  
    Offset = XIncrement * 7/20



    # 2️⃣ Calcular incrementos para el marco (más denso)
    NX_Frame = NX * FrameDensityFactor
    NY_Frame = NY * FrameDensityFactor
    FrameXIncrement = FrameSize / (NX_Frame - 1)
    FrameYIncrement = FrameSize / (NY_Frame - 1)



     # 4️⃣ Generar coordenadas del marco
    FrameHorizontal = []
    FrameVertical = []

    # 🔹 Bandas izquierda y derecha
    for i in range(int(NX_Frame)):
        if i == 1:
            x = CenterOfMass[0] - (XWidth / 2 + FrameSize) + i * FrameXIncrement + XOffset - Offset
        else :
            x = CenterOfMass[0] - (XWidth / 2 + FrameSize) + i * FrameXIncrement + XOffset
        FrameHorizontal.append(x)
        FrameHorizontal.append(x + XWidth + FrameSize + Offset )

    # 🔹 Bandas superior e inferior
    print("XIncrement",XIncrement)
    for i in range(int(NY_Frame)):
        print(f"i = {i} ")
        if i == 1:
            y = CenterOfMass[1] - (YHeight / 2 + FrameSize) + i * FrameYIncrement + YOffset - Offset 
        else:    
            y = CenterOfMass[1] - (YHeight / 2 + FrameSize) + i * FrameYIncrement + YOffset
    
        print("y ", y)
        FrameVertical.append(y)

        FrameVertical.append(y + YHeight + FrameSize + Offset )

    # Combinar las listas
    HorizontalCoordinates.extend(FrameHorizontal)
    VerticalCoordinates.extend(FrameVertical)


    return HorizontalCoordinates, VerticalCoordinates, PlanesDimTags


def createCuttingPlanes(CenterOfMass, lineLenght=200):
    """
                    Adding points to surface
    """
    _, VerticalCoordinates, PlanesDimTags = getCoordinates(CenterOfMass) # only want vertical coordinates
    LineTags = []
    count = 0
    count_2 = 0
    for Y in VerticalCoordinates:
        HorizontalCoordinates, _, _= getCoordinates(CenterOfMass) # drawing grid with horizontal
        # print(HorizontalCoordinates)
        HorizontalCoordinates_ = evaluate_coordinate(count, HorizontalCoordinates) # evaluate coordinates, I want this coord?
        for X in HorizontalCoordinates:
            for X_ in HorizontalCoordinates_:
                # print(f"X: {X}; X_:{X_}")
                if X == X_:
                    PLow = factory.addPoint(X,-Y, CenterOfMass[2]-lineLenght//2) # adding low point
                    PHigh = factory.addPoint(X,-Y, CenterOfMass[2]+lineLenght//2) # adding high point
                    LineTags.append(factory.addLine(PLow,PHigh))  # drawing line between low and high point
                    count_2= 0
                    break
                else:
                    count_2=1
            if count_2==1:
                empty_value = []
                LineTags.append(empty_value) # if i dont want coordinate, i add empty value
                count_2 = 0
        count += 1
    
    print(f"LineTags: {LineTags}")
    factory.synchronize()
    return PlanesDimTags, LineTags

def getPointsOnSurface(DimTag, LineTags):
    '''
        get points on surface that intersect with the cutting planes
        These points are contact points for the pipes
    '''
    Outs = []
    for LineTag in LineTags: 
        if LineTag!=[]:
            # verify if surface is intersected by line
            IntersectOut = factory.intersect([DimTag], [(1,LineTag)], removeObject=False)
            print(f"IntersectOut: {IntersectOut}")
            Outs.append(IntersectOut[0])
        else:
            # if no intersection, add empty value
            IntersectOut = []
            Outs.append(IntersectOut)
            print(f"IntersectOut: {IntersectOut}")
        
    print(f'Outs: {Outs}')
    
    factory.synchronize()
    Coords = []
    Mask = []
    print(Mask)

    for Out in Outs:
        if Out != []:
            BoundaryPoints = gmsh.model.getBoundary(Out)
            P1 = BoundaryPoints[0] ## p low
            P2 = BoundaryPoints[1] ## p high
            P2Coords = gmsh.model.getValue(0, P2[1], []) ## get coordinates of p high
            Coords.append(P2Coords) ## add coordinates to list of points of contact
            print(f"Coords: {P2Coords}")
            Mask.append(1)
        else:
            Coords.append([0,0,0])
            Mask.append(0)

    # this is for fixing some mask values
    for i in range(len(Mask)):
        try:
            if Mask[i]==1 and Mask[i+1]==0 and Mask[i+2]==1:
                Mask[i+1] = 1
        except:
            pass
    print(len(Mask))
    print(Mask)
    print(f"Longitud de Mask: {len(Mask)}")
    # if len(Mask) != 121:  # Si el tamaño no es 144
    #     print(f"Advertencia: la longitud de Mask es {len(Mask)}. Se ajustará.")
    #     # Rellenar con ceros o con cualquier valor predeterminado que desees
    #     Mask = np.ones(144)  # Esto garantiza que Mask tenga 144 elementos

    Mask = np.reshape(Mask, (ny,nx))
    
    return Outs, Coords, Mask
        
print("check 1 ") 
def createPointsFromCuts(PlanesDimTags, DimTag):
    PointsLists = []
    
    for (i, PlaneDimTag) in enumerate(PlanesDimTags):        
        print(f'PlaneDimTag: {PlaneDimTag}')

        IntersectOut = factory.intersect([DimTag], [PlaneDimTag], removeObject=False)
        IntersactSurfaceDimTag = IntersectOut[0][0]
        print(f'IntersectOut: {IntersectOut}')
        factory.synchronize()
        BoundaryPointsSurface = model.getBoundary([IntersactSurfaceDimTag], recursive=True)
        CoordinatesBoundaryPoints = np.empty((0,3))
        print(f'BoundaryPointsSurface: {BoundaryPointsSurface}')

        for PointTag in BoundaryPointsSurface:
            Coordinates = model.getValue(0, PointTag[1],[])
            CoordinatesBoundaryPoints = np.append(CoordinatesBoundaryPoints, [Coordinates],0)

        print(f'CoordinatesBoundaryPoints: {CoordinatesBoundaryPoints}')
        
        PointsLists.append(CoordinatesBoundaryPoints)
    return PointsLists

factory.synchronize()
gmsh.merge(FileName)
DimTag = (3,1)

CenterOfMass = calcCenterOfMassObject(DimTag)
rotate_model(np.pi)


BoxSide = 250
ZOffset = 2
BoxDimTag = (3, factory.addBox(CenterOfMass[0]-BoxSide/2, CenterOfMass[2]-BoxSide/2, CenterOfMass[2]-ZOffset, BoxSide, BoxSide, -BoxSide))


CutOut = factory.cut([DimTag],[BoxDimTag])
DimTag = CutOut[0][0]
print(f'DimTag: {DimTag}')

factory.synchronize()
launchGUI()

PlanesDimTags, LineTags = createCuttingPlanes(CenterOfMass)

factory.synchronize()
launchGUI()
print("check") 

Out, PointsOnSurfaceCoordinates, Mask = getPointsOnSurface(DimTag, LineTags)

def ordenar_lista(lista):
    return sorted(lista, key=lambda item: (round(float(item[0]), 8), round(float(item[1]), 8)))



# Ordenar por X y luego por Y
PointsOnSurfaceCoordinates = ordenar_lista(PointsOnSurfaceCoordinates)
print("Lista ordenada por X y luego por Y:", PointsOnSurfaceCoordinates)

np.save("Domos/PointsOnSurface.npy", PointsOnSurfaceCoordinates)
np.savetxt("Domos/PointsOnSurface.txt", PointsOnSurfaceCoordinates)

np.savetxt("Domos/Mask.txt", Mask.astype(int), fmt="%i")