# Control

Files relating to the sensing, calulcation and actuation of the robot are contained in this folder.
 
## Control File Descriptions
 
#### Aurora.py
 
This files uses the Tracking class within Tracker.py to every 0.05 seconds clear the previous content of the current position data and write the current.

#### Tracking.py

The class that is used by Aurora.py in order to properly calcuate the distance of the tracking sensor from the main reference sensor.

#### actuatePumps.ino

This file takes uses firmata and string callbacks to process the volume to be actuated into the robots 3 chambers and does so using the analogwrite at the end of the stringDataCallback.

#### simplePID.py

The simple_PID module is used as the model for our PID calculations. Within main, the current position of the driver simulation coupled with the real position of the aurora sensor is used to find the corresponding position. This position is then used for processing elsewhere. 

# Driver Scene

Files relating to the SOFA simulation that is used control the simulation. In both driver and shotgun folders, there are additional folders containing mesh files for this model. Also, only MultiController.py and scene.py have been modified when comparing the two. Connexion_Function_bos.py just loads in the mesh model for the simulation and Collapsible_Actuator_Function.py again loads in the mesh file and then defines the beam of nodes structure that is the simulation.

## Driver File Descriptions

#### Scene.py - Driver
The generic GoalShift(rootNode) method, we are able to simply drag and drop the position of the head of the robot in the SOFA scene. 

#### MuliController.py - Driver
This file contains all my experimental control methods, however GoalShift is in fact the only one that is needed to drive the file

# Shotgun Scene

Files relating to the SOFA simulatation that is used to take the calcualted PID value in coordinates, and output the correct corresponding chamber volumes. Again, only the scene and multicontroller files have been modified for this model and there is mesh folder with all models of the robot structure. 

## Shotgun File Descriptions

#### Scene.py - Shotgun

During the creation of the actuator we select goToPositionFromCSV(RootNode = rootNode, x = 14.0, y = 5.0, z = 2.0), note that the start points of 14,5,2 are based off the current mesh model dimensions but can be changed and deleted within the method. Note that we will also be using VolumePrinter(module = collapsible,RootNode = rootNode) to send the volume to be actuated into each chamber is sent via firmata to the actuatepumps.ino file. 

#### MuliController.py - Shotgun

The method of control used in this file is located at the very end and uses the class goToPositionFromCSV(Sofa.Core.Controller). This class records prev_position is as the current position at the start of the loop, reads in the positions to be converted from Simple_PID.py or the convert_pos2vol.csv file, processes the string to make break it into its componenet parts and then in one big movement the robot is sent to that given position within the onAnimateBeginEvent Callback used here. Note that the volume printer is used to send the volume to be actuated into each chamber via firmata to the actuatepumps.ino file. 

