import gmsh
import numpy as np
factory = gmsh.model.occ

gmsh.initialize()
gmsh.model.add("step_model")

# Cargar archivo .step

# Cargar puntos desde archivo .npy
points = np.load("Domos/PointsOnSurface.npy")

# Parámetro: radio de las esferas
radio = 0.9 # puedes ajustar este valor

# Agregar una esfera por cada punto
for i, (x, y, z) in enumerate(points):
    gmsh.model.occ.addSphere(x, y, z, radio, tag=i+2)

# Sincronizar para que se reflejen los cambios
gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.model.mesh.refine()
gmsh.model.mesh.generate(2)

# gmsh.model.occ.importShapes("Scenes/Geometries/Dome_V2_StepAbril.step")

gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.model.mesh.refine()


factory.synchronize()
gmsh.fltk.run()
gmsh.write("Domos/Esferas.step")

gmsh.finalize()