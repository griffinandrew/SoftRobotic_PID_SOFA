import Sofa
from math import sin,cos, sqrt, acos
import array
from MultiController import *
from Collapsible_Actuator_Function import *

init_pressure_value = 0 # 
h_effector = 14  # height of the end effector (the top of the robot), in mm
min_pressure = -10 # minimal pressure
max_pressure = 20 # maximal pressure   

act_flag = 0 # actuation flag 0 => inverse ; 1 => direct

pressure_step = 1 # kPa step for direct control

goal_pas = 1 # mm displacement for goal point control with keyboard (for inverse control)

# Actuators parameters
masse_module = 0.01 # not used here, this simulation is without gravity
nb_cavity = 3  # number of cavity
YM_soft_part = 15.56 # young modulus of the soft part (kPa)
coef_poisson = 0.3870 # Poisson ration
nb_poutre = 9 # number of beam
thickness = 0.025 #(mm), equivalent to 25 um
radius = 4 # radius of the beam, that is representing the actuator

chamber_model =  '15_Inside.stl'
name_cavity = 'Bellow'

def MyScene(rootNode,YM_soft_part,coef_poi,act_flag,thickness):

    collapsible = Collapsible_Actuator(init_pressure_value,YM_soft_part,coef_poisson,nb_cavity,chamber_model,max_pressure,name_cavity,masse_module,min_pressure,nb_poutre,thickness)

    # Only used to launch sofa from python scripts (to help SOFA to find the plugins)
   # rootNode.addObject('AddPluginRepository', path = 'C:/Users/19548/SOFA/SOFA_v22.06.00_Win64/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    #rootNode.addObject('AddPluginRepository', path = 'C:/Users/19548/SOFA/SOFA_v22.06.00/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
   # rootNode.addObject('AddPluginRepository', path = 'C:/Users/19548/SOFA/SOFA_v22.06.00_Win64/plugins/BeamAdapter/lib/')#/libBeamAdapter.so 1.0

    # required plugins:
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaConstraint')
    rootNode.addObject('RequiredPlugin', name='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralAnimationLoop')
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='SofaSparseSolver')
    rootNode.addObject('RequiredPlugin', name='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralLoader')
    rootNode.addObject('RequiredPlugin', name='SofaMiscFem')    
    
    #visual dispaly
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('BackgroundSetting', color='0.28 0.34 0.93') # color='0 0.168627 0.211765'

    rootNode.findData('dt').value= 0.05;
    
    rootNode.addObject('FreeMotionAnimationLoop')
    
        
    rigidFramesNode  = rootNode.addChild('RigidFrames')
    rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    rigidFramesNode.addObject('SparseLDLSolver', name='ldlsolveur')    
    # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  
    rigidFramesNode.addObject('GenericConstraintCorrection')    # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  

    ## Creation of the beam
    rigidFramesNode.addObject('RegularGrid',  name='meshLinesCombined',  nx=nb_poutre, ny='1', nz='1', xmax=h_effector, xmin='0.0', ymin='0', ymax='0',zmin='0',zmax='0')
    rigidFramesNode.addObject('MechanicalObject',  name='DOFs', template='Rigid3d', showObject='1', showObjectScale='1')
    rigidFramesNode.addObject('BeamInterpolation', name='BeamInterpolation', printLog = '1', defaultYoungModulus=YM_soft_part, dofsAndBeamsAligned='true', straight='1', crossSectionShape='circular', radius=radius)
    rigidFramesNode.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='0', massDensity=0.001)
    rigidFramesNode.addObject('RestShapeSpringsForceField', name='anchor', points='0', stiffness='1e12', angularStiffness='1e12')
      
    ## Creation of the actuator  
    actuators = collapsible.createRobot(parent = rigidFramesNode, name = "MyActuators", act_flag = act_flag)

    if act_flag == 0 :
        rootNode.addObject('QPInverseProblemSolver') #inverse
                    #goal
        goal = rootNode.addChild('goal')
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject('CGLinearSolver', iterations='1000',threshold="1e-5", tolerance="1e-5")
        goal.addObject('MechanicalObject', name='goalMO', position=[h_effector+0, 0,0])
        goal.addObject('SphereCollisionModel', radius='0.5')#, group='1')
        goal.addObject('UncoupledConstraintCorrection')

        goal2 = rootNode.addChild('goal2')
        goal2.addObject('EulerImplicitSolver', firstOrder=True)
        goal2.addObject('CGLinearSolver', iterations='1000',threshold="1e-5", tolerance="1e-5")
        goal2.addObject('MechanicalObject', name='goalM2', position=[h_effector+3, 0,0])
        goal2.addObject('SphereCollisionModel', radius='0.5')#, group='1')
        goal2.addObject('UncoupledConstraintCorrection')

        controlledPoints = rigidFramesNode.addChild('controlledPoints')
        controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3d",position=[[h_effector, 0, 0,0,0,0,1]])#,rotation=[0, 90 ,0])
        controlledPoints.addObject('PositionEffector',name = 'PositionEffector', template="Rigid3d", indices='0', effectorGoal="@../../goal/goalMO.position")
        controlledPoints.addObject('SubsetMapping', mapForces=False, mapMasses=False)


        # rootNode.addObject(GoalKeyboardController(goal_pas,rootNode)) # Choose between GoalKeyboardController and GoalShift => you may only use one, uncomment the one you want to use
        rootNode.addObject(GoalShift(rootNode))
        rootNode.addObject(positionPrinter(module = collapsible,RootNode = rootNode))

        #rootNode.addObject(PositionPrinterCsv(RootNode = rootNode, module = "C:\POSITIONS\pos"))
        # rootNode.addObject(CircleTrajectory(RootNode = rootNode, rayon = 4, nb_iter = 100))
        #rootNode.addObject(PressurePrinter(module = collapsible,RootNode = rootNode))
       # rootNode.addObject(ArduinoPressure(module = collapsible,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware)
      #  rootNode.addObject(VolumePrinter(module = collapsible,RootNode = rootNode))
        #rootNode.addObject(goToPositionFromCSV(RootNode = rootNode, x = 14.0, y = 5.0, z = 2.0))
       # rootNode.addObject(goToPositionSMALL(RootNode = rootNode, x = 10, y = 5, z = 2, nb_iterations=20))
       # rootNode.addObject(goToPosition(RootNode = rootNode, x = 10, y = 5, z = 2))

        #rootNode.addObject(goToPositionFromLast(RootNode = rootNode, x = 14.0, y = 5.0, z = 2.0))
        #rootNode.addObject(goToPositionFromLast(RootNode = rootNode, x = 9, y = 5, z = 2))
       # rootNode.addObject(goToPositionFromLast(RootNode = rootNode, x = 8, y = 5, z = 2))
       # rootNode.addObject(goToPositionFromLast(RootNode = rootNode, x = 7, y = 5, z = 2))



       # rootNode.addObject(GoalPrinterCsv(RootNode = rootNode, module = "C\VOLUME\vols"))
        #rootNode.addObject(goToPositionWithTimeStep(RootNode = rootNode, x = 10, y = 5, z = 2))

    elif act_flag == 1 :
        rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')
        # rootNode.addObject(pressureControl(rootNode)) 
        #rootNode.addObject(VolumePrinter(module = collapsible,RootNode = rootNode))
       # rootNode.addObject(PressureController(RootNode=rootNode,pressure_step=pressure_step,max_pressure=max_pressure,module=collapsible))
       # rootNode.addObject(ArduinoPressure(module = collapsible,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware)
       # rootNode.addObject(ArduinoVolume(module = collapsible,RootNode = rootNode))
       # rootNode.addObject(goToPosition(RootNode = rootNode, x = 10, y = 5, z = 2))
    return rootNode


#def passParameter():
    #potentially get the user input? otherwise idk how to update this function

    #actually i think that a better idea is to read input positions from a file and then pass them in one at a time, possibly with a delay 
    #i am confused about how exactly to pump in all the values once the beam has been created

     #open the file with reading permissions 
    #file = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "r", newline='')
    
  #  for line in file:
  #      para_to_pass = file.readline() 




    
def createScene(rootNode):
    MyScene(rootNode,YM_soft_part=YM_soft_part,coef_poi = coef_poisson,thickness = thickness,act_flag = act_flag) # act_flag = 0- IP ; 1- Direct Control
