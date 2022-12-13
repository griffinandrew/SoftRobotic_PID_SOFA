#BU MBL 11/28/22 
#extracting data from arduino and sending to csv for processing in simplePID file

from Tracker import Tracking
import time
import csv
from sksurgerynditracker.nditracker import NDITracker

# Initialize settings for NDI library -- DO NOT CHANGE anything except port numbers when necessary
SETTINGS = {
    "tracker type": "aurora",
    "ports to use": [1, 2]
}
TRACK = NDITracker(SETTINGS)
TRACK.start_tracking()
port_handles, timestamps, framenumbers, tracking, quality = TRACK.get_frame()



while True:
   # print('running')
    # Introduce a small-time delay (otherwise all values become undefined)

    time.sleep(0.05)
    #truncation / clearing of file
    f = open("C:\Driver_Positions\Aurora_pos.csv", "w", newline='')
    f.truncate()
    f.close() 
    

    # Call an instance of TRACKER.get_frame to populate "matrix" with data from the sensors
    matrix = [TRACK.get_frame()]

    # Call in the Tracking class as "tt"
    tt = Tracking()

    # Read out the Euclidean coordinates of sensors 1 & 2
    [x1, y1, z1, x2, y2, z2] = tt.extractposition(matrix)

    # # Print out the Euclidean coordinates of sensors 1 & 2
    # print(x1, y1, z1, x2, y2, z2)

    # New coordinates of sensor 2 when sensor 1 is defined as origin
    x3 = x2 - x1
    y3 = y2 - y1
    z3 = z2 - z1

    #writing coordinates to file for export
    f = open("C:\Driver_Positions\Aurora_pos.csv", "w", newline='')

    writer = csv.writer(f)
    
    vector = [round(x3, 3), round(y3, 3), round(z3, 3)]

    writer.writerow(vector)
    f.close()


    ts = time.time()
    print(round(x3, 3), round(y3, 3), round(z3, 3), ts, '\n')
