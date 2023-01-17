 ## Control
 
 Files relating to the sensing, calulcation and actuation of the robot are contained in this folder
 
 #### Aurora.py
 
This files uses the Tracking class within Tracker.py to every 0.05 seconds clear the previous content of the current position data and write the current

#### Tracking.py

The class that is used by Aurora.py in order to properly calcuate the distance of the tracking sensor from the main reference sensor

#### actuatePumps.ino

This file takes uses firmata and string callbacks to process the volume to be actuated into the robots 3 chambers and does so using the analogwrite at the end of the stringDataCallback

#### simplePID.py

The simple_PID module is used as the model for our PID calculations. Within main, the current position of the driver simulation coupled with the real position of the aurora sensor is used to find the corresponding position. This position is then used for processing elsewhere. 

