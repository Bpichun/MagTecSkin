import gmsh 
import numpy as np
import ConstantsRigid_DomoPequeño as cons


factory = gmsh.model.occ
launchGUI = gmsh.fltk.run
gmsh.fltk.initialize()

def createLines(PointTags, Close=True):
    
    LineTags = np.empty((0,1),dtype=int)
    NPoints = len(PointTags)
    for i in range(1, NPoints):
        LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[i-1], PointTags[i])])
    
    if Close:
        LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[-1], PointTags[0])])
    
    #print('LineTags: ' + str(LineTags))
    return LineTags  

gmsh.initialize()



P0 = factory.addPoint(cons.DomeRadiusBase,0,0)
P1 = factory.addPoint(cons.DomeRadiusBase,0,cons.Offset)
P2 = factory.addPoint(cons.DomeRadiusSec1,0,cons.Sec1Height)
P3 = factory.addPoint(cons.DomeRadiusSec2,0,cons.Sec2Height)
P4 = factory.addPoint(cons.Offset3+cons.Offset,0,cons.DomeHeight)
P5 = factory.addPoint(cons.Offset3,0,cons.DomeHeight)

P0Line = factory.addPoint(cons.DomeRadiusBase+(cons.WallThickness/2)-1,0,0) # - a +
P1Line = factory.addPoint(cons.DomeRadiusBase-cons.WallThickness/2,0,0) # + a -


ExtrusionLine = factory.addLine(P0Line, P1Line)
factory.synchronize()
launchGUI()

PointTags = [P0,P1,P2,P3,P4,P5]

# LineTags = createLines(PointTags, Close=False)

SplineTag = factory.addBSpline(PointTags)
WireTag = factory.addWire([SplineTag])

ProfileDimTags = factory.addPipe([(1,ExtrusionLine)],WireTag)



print("launch 1 ")

factory.synchronize()
# launchGUI()

########### Cilindros en la normal : ###############################################################
curve = 3 # 5 a 3
Bounds = gmsh.model.getParametrizationBounds(1,curve)
print("Bounds", Bounds) #Parametrizacion de 0 a 3

x, y, z = gmsh.model.getValue(1, curve, [0.5])
print("Coordenadas:", x, y, z)

# gmsh.model.getNormal(5, [0.5]) No funciona en curvas



u = 1.5  # Punto intermedio

# Parámetros
num_cilindros = 5  # Número de cilindros a lo largo de la curva
radius = 1.5       # Radio del cilindro
length = 1        # Longitud del cilindro

Bound_Max = 3

Lista_Cilindros = []
# Generar cilindros en varios puntos de la curva
for u in np.linspace(0, Bound_Max, num_cilindros):  # Valores equiespaciados en [0,1]
    # Obtener el punto en la curva
    x, y, z = gmsh.model.getValue(1, curve, [u])
    print("x, y, z",x, y, z)

    # Obtener el vector tangente
    Tx, Ty, Tz = gmsh.model.getDerivative(1, curve, [u])

    # Calcular el vector normal en XZ
    Nx, Ny, Nz = -Tz, 0, Tx  # Rotar 90° en el plano XZ

    # Normalizar la normal
    norm = np.sqrt(Nx**2 + Ny**2 + Nz**2)
    Nx, Ny, Nz = Nx / norm, Ny / norm, Nz / norm

    # Escalar la normal para definir la longitud del cilindro
    Nx, Ny, Nz = Nx * length, Ny * length, Nz * length

    # Agregar el cilindro a la lista
    Lista_Cilindros.append((x, y, z, Nx, Ny, Nz, radius))

# Crear los cilindros en Gmsh
indices = [1, 3]
angulos = [90, 180, 270]  # Ángulos de rotación en grados
copias = []
cilindros_ids = []

for i in indices:
    cid = gmsh.model.occ.addCylinder(*Lista_Cilindros[i])
    cilindros_ids.append((3, cid))  # Guardar referencia (3, id) para entidades 3D

# Rotar y copiar cilindros a diferentes ángulos
for angulo in angulos:
    radianes = np.radians(angulo)  # Convertir a radianes
    for cid in cilindros_ids:
        copia = gmsh.model.occ.copy([cid])  # Copiar cilindro
        gmsh.model.occ.rotate(copia, 0, 0, 0, 0, 0, 1, radianes)  # Rotar sobre Z

# Sincronizar geometría
gmsh.model.occ.synchronize()




gmsh.model.occ.synchronize()


print("launch 1.2 ")
factory.synchronize()
# launchGUI()
print("exit")
# exit()

#################################################################################


P6 = factory.addPoint(cons.DomeRadiusBase,cons.DomeRadiusBase+cons.WallThickness/2,0)
CaveLine = factory.addLine(P0,P6)
Wire2Tag = factory.addWire([CaveLine])

Profile2DimTags = factory.addPipe(ProfileDimTags, Wire2Tag, "DiscreteTrihedron")

# MagnetRadius = 1
# MagnetHeight = 4

# Epsilon = 0.5
# CylinderDimTag = (3,factory.addCylinder(0,0,DomeHeight-WallThickness/2-Epsilon, 0,0, MagnetHeight,MagnetRadius))
# print("Profile2DimTags:", Profile2DimTags)
# print("Type of Profile2DimTags:", type(Profile2DimTags))

# CutOut = factory.cut(Profile2DimTags,[CylinderDimTag])
# Profile2DimTags = CutOut[0]

# Eliminar ProfileDimTags después de usarlo
factory.remove(ProfileDimTags,recursive=True)
del ProfileDimTags
factory.synchronize()
print("launch 2 ")

gmsh.fltk.awake(action="update")
launchGUI()

BoxDimTag = (3,factory.addBox(0,0,0,np.sqrt(2)*(cons.DomeRadiusBase+cons.WallThickness/2),np.sqrt(2)*(cons.DomeRadiusBase+cons.WallThickness/2),np.sqrt(2)*(cons.DomeRadiusBase+cons.WallThickness/2)))

factory.rotate([BoxDimTag],0,0,0,0,0,1,np.pi/4)

CutOut = factory.cut(Profile2DimTags,[BoxDimTag])
DomeEighthDimTag = CutOut[0][0]
DomeEighthCopyDimTags = factory.copy([DomeEighthDimTag])
factory.symmetrize([DomeEighthDimTag],1,-1,0,0)

FuseOut = factory.fuse([DomeEighthDimTag],DomeEighthCopyDimTags)
FuseDimTag = FuseOut[0][0]



factory.synchronize()
print("launch 2.1 ")
# launchGUI()

# create the magnet



CylinderDimTag = (3,factory.addCylinder(0,0,cons.DomeHeight-cons.WallThickness/2-cons.Epsilon, 0,0, cons.MagnetHeight,cons.MagnetRadius))

CutOut = factory.cut([FuseDimTag],[CylinderDimTag])
FuseOut = CutOut[0][0]


factory.synchronize()
print("launch 3 ")

# launchGUI()
# exit()
# ########### Cilindros Diagonales : ###############################################################
curve = 36 # 49 a 36

Bounds = gmsh.model.getParametrizationBounds(1,curve)
print("Bounds", Bounds) #Parametrizacion de 0 a 3

# # x, y, z = gmsh.model.getValue(1, 5, [0.5])
# # print("Coordenadas:", x, y, z)

# # gmsh.model.getNormal(5, [0.5]) No funciona en curvas



u = 0.5  # Punto intermedio

# Parámetros
num_cilindros = 7  # Número de cilindros a lo largo de la curva
# radius = 1.5       # Radio del cilindro
# length = 0.5        # Longitud del cilindro

Bound_Max = float(Bounds[1])

Lista_Cilindros = []
# Generar cilindros en varios puntos de la curva
for u in np.linspace(Bound_Max, 0, num_cilindros):  # De atrás hacia adelante
  # Valores equiespaciados en [0,1]
    # Obtener el punto en la curva
    x, y, z = gmsh.model.getValue(1, curve, [u])
    print("x, y, z",x, y, z)
    # Obtener el vector tangente
    Tx, Ty, Tz = gmsh.model.getDerivative(1, curve, [u])

    # # Calcular el vector normal en XZ
    # Nx, Ny, Nz = Tz, 0, -Tx  # Rotar 90° en el plano XZ

    # # Normalizar la normal
    # norm = np.sqrt(Nx**2 + Ny**2 + Nz**2)
    # Nx, Ny, Nz = Nx / norm, Ny / norm, Nz / norm

    # # Escalar la normal para definir la longitud del cilindro
    # Nx, Ny, Nz = Nx * length, Ny * length, Nz * length


    # gmsh.model.occ.addCylinder(x, y, z, Nx, Ny, Nz, radius)

        # Agregar el cilindro a la lista
    Lista_Cilindros.append((x, y, z, Tx, Ty, Tz, radius))



v = np.array([1, 1, 0])
v_normalizado = v / np.linalg.norm(v)

indices = [1, 3]
angulos = [90, 180, 270]  # Ángulos de rotación en grados
copias = []
cilindros_ids = []

for i in indices:
    x, y, z, Tx, Ty, Tz, radius = Lista_Cilindros[i]
    target = np.array([Tx, Ty, Tz])
    target_normalizado = target / np.linalg.norm(target)
    cos_theta = np.dot(v_normalizado, target_normalizado)
    print("cos_theta",cos_theta)

    theta_rad = np.arccos(cos_theta)   
    cilindro_tag = gmsh.model.occ.addCylinder(0, 0, -1, 0, 0, 1, radius)
    gmsh.model.occ.rotate([(3, cilindro_tag)],   # (dim=3, tag)
                            0, 0, 0,               # punto de rotación (el origen)
                            0, 1, 0,               # eje de rotación (x)
                            theta_rad)             # ángulo en radianes
    gmsh.model.occ.rotate([(3, cilindro_tag)],    # el cilindro
                            0, 0, 0,                # centro de rotación
                            0, 0, 1,                # eje Z
                            np.radians(45))          # ángulo en radianes
        
    gmsh.model.occ.translate([(3, cilindro_tag)],  # (dim=3, tag)
                               x, y, z)               # Desplazamiento (x, y, z)
    
    # Rotación en X (con ángulo theta_rad)
    rot_x = np.array([[1, 0, 0],
                    [0, np.cos(theta_rad), -np.sin(theta_rad)],
                    [0, np.sin(theta_rad), np.cos(theta_rad)]])

    # Rotación en Z (con 45 grados)
    rot_z = np.array([[np.cos(np.radians(45)), -np.sin(np.radians(45)), 0],
                    [np.sin(np.radians(45)), np.cos(np.radians(45)), 0],
                    [0, 0, 1]])
    # direction = np.array([0, 0, 1])
    # final_direction = np.dot(rot_z, np.dot(rot_x, direction))
    # print("Dirección final:", final_direction)
    cilindros_ids.append((3, cilindro_tag))

# Rotar y copiar cilindros a diferentes ángulos
for angulo in angulos:
    radianes = np.radians(angulo)  # Convertir a radianes
    for cid in cilindros_ids:
        copia = gmsh.model.occ.copy([cid])  # Copiar cilindro
        gmsh.model.occ.rotate(copia, 0, 0, 0, 0, 0, 1, radianes)  # Rotar sobre Z


# print("launch 3.1 ")
# factory.synchronize()
# launchGUI()
# print("exit")
# exit()


# Posición inicial
x, y, z = 0.0, 0.0, 27.5 
dx, dy, dz = 0.0, 0.0, -(length) 
# radius = 1.0
# length = 
cilindro_tag = gmsh.model.occ.addCylinder(x, y, z, dx, dy, dz, radius)

# # Sincronizar geometría
# gmsh.model.occ.synchronize()




# gmsh.model.occ.synchronize()


print("launch 3.2 ")
factory.synchronize()
launchGUI()
# print("exit")
# exit()

# #################################################################################


Copy1Fuse = factory.copy([FuseDimTag])
factory.symmetrize(Copy1Fuse,1,0,0,0)
Copy2Fuse = factory.copy([FuseDimTag])
factory.symmetrize(Copy2Fuse,0,1,0,0)
Copy3Fuse = factory.copy([FuseDimTag])
factory.symmetrize(Copy3Fuse,0,1,0,0)
factory.symmetrize(Copy3Fuse,1,0,0,0)

factory.fuse([FuseDimTag],Copy1Fuse+Copy2Fuse+Copy3Fuse)

factory.remove([(1, SplineTag)])


factory.synchronize()
launchGUI()
gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()
gmsh.model.mesh.generate(3)
factory.synchronize()
launchGUI()
gmsh.write("Dome_DomoPequeño.stl")
gmsh.write("Dome_DomoPequeño.vtk")
gmsh.write("Dome_DomoPequeño.step")
gmsh.write("Dome_DomoPequeño.stp")
gmsh.fltk.run()
