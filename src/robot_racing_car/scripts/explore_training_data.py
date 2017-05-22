#!/usr/bin/env python

import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt


csv_lines = []
with open("../training_data/training_data.csv") as csvfile:
	reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for line in reader:
		csv_lines.append(line)

steering_commands = []

for csv_line in csv_lines:
	steering_command = int(csv_line[1])

	source_path = "../training_data/" + csv_line[0]

    steering_commands.append(steering_command)

steering_commands = np.array(steering_commands)

bins=[0, 1, 2, 3, 4, 5]

plt.hist(steering_commands, bins, align="left")
plt.xticks([0, 1, 2, 3, 4], ["Forward", "Back", "Left", "Right", "Break"])
plt.show()
