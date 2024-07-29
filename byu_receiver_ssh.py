'''
This file contains the main functions to receive data from the sensor array.
This file is not compatible with python 3.12. Use 3.11.
'''

# Import dependencies
import serial  # Communication with the sensors pip install pyserial
import numpy as np  # Linear algebra and data management
import matplotlib.pyplot as plt  # Plotting
import pandas as pd
import os
import datetime as date
import RANSAC as RS  # Import the RANSAC code
from Parameters import *

# Change the serial port configuration for Linux
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=3)

# Delimiters:
# Data is transmitted in the form ###c###p###c###l###c###p###c###
delim_coord = "c"
delim_point = "p"
delim_line = "l"

print("Starting")

x_current = x_step

# Create empty lists for the sensor banks
right_bank = []
left_bank = []

# Empty list for all of the data
all_points = []
# Create empty lists for RANSAC output
segments = []

def parse_points(line):
    '''Take the raw line from a request and convert it into a list of points'''
    # Separate the lines into lists of points (Still strings)
    line = line.split(delim_point)[0:-1]  # Drop the last entry since it is empty (see Arduino Code)

    # Create an empty list to store the points to return
    points = []

    # Parse the strings into points
    for entry in line:
        pt = entry.split(delim_coord)
        if abs(float(pt[1])) > .28:
            points.append(np.array([float(pt[0]), float(pt[1])]))  # Parse the coordinates into floats and save them

    # Add the new points to the full list
    global all_points
    all_points = all_points + points

    # Remove points in the center band and outside the valid zone.
    len_original = len(points)
    for i in range(len(points)):
        if abs(points[len_original-i-1][1]) < .28 or abs(points[len_original-i-1][1]) > 2.15:
            points.pop(len_original-i-1)
    return points

# Send the set speed to the sensors
string = str(speed)
ser.write(string.encode('utf-8'))

# Main loop
try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            right_new = []
            left_new = []

            # Parse the points
            points = parse_points(line)

            # Allocate points to banks
            for p in points:
                if p[1] < 0:
                    right_new.append(p)
                else:
                    left_new.append(p)

            # Determine if we are ready for RANSAC
            if abs(x_current - points[0][0]) > x_step:
                # Perform RANSAC
                if len(right_bank) > 1 and len(left_bank) > 1:
                    right_line = RS.fit_line(right_bank)
                    left_line = RS.fit_line(left_bank)
                    segments.append(right_line)
                    segments.append(left_line)

                # Increment window
                x_current += x_step

                # Clear the figure and plot segments (Commented out for SSH compatibility)
                # plt.clf()
                # Arrays of current points
                rb = np.array(right_bank)
                lb = np.array(left_bank)

                # for i in segments:
                #     RS.plot_segment(i)
                ap = np.array(all_points)
                # plt.scatter(ap[:,1], ap[:,0])
                # plt.scatter(rb[:,1], rb[:,0])
                # plt.scatter(lb[:,1], lb[:,0])
                # plt.draw()
                # plt.pause(.01)

                # Reset the old points
                right_bank = []
                left_bank = []

            # After RANSAC, or if RANSAC wasn’t ready, add new points to the sensor banks
            right_bank = right_bank + right_new
            left_bank = left_bank + left_new

except KeyboardInterrupt:
    print("Stopped")

# Save the data
now = date.datetime.now()
test_dir = os.path.join(os.getcwd(), "Tests", f"{now.month}-{now.day}-{now.year} {now.hour}-{now.minute}-{now.second}")
os.mkdir(test_dir)

segments_df = pd.DataFrame(columns=["x1", "y1", "x2", "y2"])
for s in segments:
    segments_df.loc[len(segments_df)] = list(np.concatenate(s, axis=0))
segments_df.to_csv(os.path.join(test_dir, "Segments.csv"))

point_df = pd.DataFrame(columns=["x", "y"])
for p in all_points:
    point_df.loc[len(point_df)] = list(p)
point_df.to_csv(os.path.join(test_dir, "Points.csv"))

# Save the figure as a pdf (Commented out for SSH compatibility)
# plt.clf()
# for i in segments:
#     RS.plot_segment(i)
# ap = np.array(all_points)
# plt.scatter(ap[:,1], ap[:,0])
# fig = plt.gcf()
# fig.savefig(os.path.join(test_dir, "figure.pdf"), format="pdf")
# plt.show()
