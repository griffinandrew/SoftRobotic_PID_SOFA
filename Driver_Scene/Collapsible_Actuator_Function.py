import Sofa.Core

# utilise Rigidify de STLIB
#from stlib3.physics.mixedmaterial import Rigidify
from os import getcwd

# permet d'initialiser un robot au nb de module voulu :
# - mettre tous les paramètres dans l'initialisation de la variable
# - appeler ensuite seulemnt createRobot qui appelle les autres fonctions, tous les paramètres étant dans l'initialisation (voir stiff_module.pyscn)

def define_mesh_path(mesh_name):
        path = getcwd()
        inter = '/mesh/'

        file_path = path + inter +mesh_name
        print("###########################")
        print(str(file_path))
        return file_path


class Collapsible_Actuator() :  

    def __init__(self,init_pressure_value,YM_soft_part,coef_poisson,nb_cavity,chamber_model,max_pressure,name_cavity,masse_module,min_pressure,nb_poutre,thickness):
        # self.h_module = h_module
        self.init_pressure_value = init_pressure_value
        self.YM_soft_part = YM_soft_part
        self.coef_poi = coef_poisson
        self.nb_cavity = nb_cavity
        self.i_cavity = 0
        self.ang_dec = 360/nb_cavity # calcul du placement des cavités
        self.chamber_model = chamber_model
        # self.module_model = module_model
        self.max_pressure = max_pressure
        self.name_cavity = name_cavity
        self.masse_module = masse_module
        self.nb_poutre = nb_poutre
        self.min_pressure = min_pressure
        self.thickness = thickness

    def createCavity(self,parent,name_c,cavity_model,act_flag): # for v1 -------
        bellowNode = parent.addChild(name_c)
        bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[90,0,self.ang_dec*self.i_cavity], translation=[0, 0,0])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
        bellowNode.addObject('MeshTopology', src='@meshLoader', name='chambreAMesh')
        bellowNode.addObject('MechanicalObject', name='chambreA',rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
        bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
        # bellowNode.addObject('SurfacePressureModel', name='SPC_model', minPressure = 0,maxPressure = 20)
        bellowNode.addObject('TriangleFEMForceField', thickness=self.thickness, youngModulus='100',poissonRatio=0.3)
        if act_flag == 0 : # if pas indispensable => IP fonctionne avec Constraint (étrangement)
            bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh.triangles',minPressure = self.min_pressure,maxPressure = self.max_pressure)#,maxPressureVariation = 20)#,valueType=self.value_type)
        elif act_flag == 1 :
            bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMess.triangles', value=self.init_pressure_value,minPressure = 0,maxPressure = self.max_pressure)#,maxPressureVariation = 20)#,valueType=self.value_type)
        # bellowNode.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )
        self.i_cavity = self.i_cavity + 1;
        if self.i_cavity == self.nb_cavity :
            self.i_cavity = 0
        return bellowNode

    # def createVolume(self,parent,name_c,i,model): ## ?? I keep it there in cas we want to add volume to the simulation
    #     module = parent.addChild('stiff_flop'+str(i+1))
    #     module.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
    #     module.addObject('SparseLDLSolver', name='directSolver' , template="CompressedRowSparseMatrixd")
    #     # module.addObject('MeshSTLLoader', filename='mesh/solid_STIFF_FLOP02_attach_s.stl', flipNormals='0', triangulate='true', name='loader' )#, translation=[4., 0,-3.275], rotation=[0,0,90])
    #     module.addObject('MeshVTKLoader', name='loader', filename=model,translation = [0,0,self.h_module*i], rotation=[0, 0 , 0]) # 
    #     module.addObject('MeshTopology', src='@loader', name='container')
    #     module.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1,rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
    #     module.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part) # stable youngModulus = 500 / réel ? = 103
    #     module.addObject('UniformMass', totalMass=self.masse_module)
    #     return module

    def createRobot(self,parent,name,act_flag):

        chamber_model_path = define_mesh_path(self.chamber_model) 

        for j in range(self.nb_cavity):
            name = 'Bellow' + str(j+1) + '1' # +'1' pour coller au formalisme et pouvoir réutiliser les fonctions de connexion sans changements
            bellowNode1 = self.createCavity(parent,name,chamber_model_path,act_flag)
            bellowNode1.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA' )
