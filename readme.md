 ## Control
 
 Files relating to the sensing, calulcation and actuation of the robot are contained in this folder.
 
 #### Aurora.py
 
This files uses the Tracking class within Tracker.py to every 0.05 seconds clear the previous content of the current position data and write the current.

#### Tracking.py

The class that is used by Aurora.py in order to properly calcuate the distance of the tracking sensor from the main reference sensor.

#### actuatePumps.ino

This file takes uses firmata and string callbacks to process the volume to be actuated into the robots 3 chambers and does so using the analogwrite at the end of the stringDataCallback.

#### simplePID.py

The simple_PID module is used as the model for our PID calculations. Within main, the current position of the driver simulation coupled with the real position of the aurora sensor is used to find the corresponding position. This position is then used for processing elsewhere. 


## Driver Scene

Files relating to the SOFA simulation that is used control the simulation. In both driver and shotgun folders, there are additional folders containing mesh files for this model. Also, only MultiController.py and scene.py have been modified when comparing the two. Connexion_Function_bos.py just loads in the mesh model for the simulation and Collapsible_Actuator_Function.py again loads in the mesh file and then defines the beam of nodes structure that is the simulation.

##### Scene.py
The generic GoalShift(rootNode) method, we are able to simply drag and drop the position of the head of the robot in the SOFA scene. 

##### MuliController.py
This file contains all my experimental control methods, however GoalShift is in fact the only one that is needed to drive the file

## Shotgun Scene

Files relating to the SOFA simulatation that is used to take the calcualted PID value in coordinates, and output the correct corresponding chamber volumes. Again, only the scene and multicontroller files have been modified for this model and there is mesh folder with all models of the robot structure. 



