# Import relevant libraries
# import math
import time
from sksurgerynditracker.nditracker import NDITracker

# # Initialize settings for NDI library -- DO NOT CHANGE anything except port numbers when necessary
# SETTINGS = {
#     "tracker type": "aurora",
#     "ports to use": [1, 2]
# }
# TRACKER = NDITracker(SETTINGS)
# TRACKER.start_tracking()
# port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()


class Tracking:

    def __init__(self):
        self.intermediate_1 = None
        self.intermediate_2 = None
        self.intermediate_3 = None

    # Define function to extract position coordinates from Aurora @ each instance of TRACKER.get_frame in Main function
    def extractposition(self, bigarray):

        # First intermediate array within bigArray
        self.intermediate_1 = bigarray[0]

        # Second intermediate array within bigArray
        self.intermediate_2 = self.intermediate_1[3]

        # Third intermediate array within bigArray -- index value corresponds with port numbers
        # Index of 0 corresponds to the sensor hooked up to port 1
        self.intermediate_3 = self.intermediate_2[0]

        # Extract Euclidean position values from intermediate_3 for sensor 1
        x1 = self.intermediate_3[0][3]
        y1 = self.intermediate_3[1][3]
        z1 = self.intermediate_3[2][3]

        # Third intermediate array within bigArray -- index value corresponds with port numbers
        # Index of 1 corresponds to the sensor hooked up to port 2
        self.intermediate_3 = self.intermediate_2[1]

        # Extract Euclidean position values from intermediate_3 for sensor 2
        x2 = self.intermediate_3[0][3]
        y2 = self.intermediate_3[1][3]
        z2 = self.intermediate_3[2][3]

        return x1, y1, z1, x2, y2, z2
        