#!/usr/bin/env python

import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt


csv_lines = []

with open("../training_data/training_data.csv") as csvfile:
	reader = csv.reader(csvfile, delimiter=",", quotechar='|')
	for line in reader:
		csv_lines.append(line)

steering_commands = []

left_count = 0
right_count = 0

for csv_line in csv_lines:
	steering_angle = float(csv_line[1])
	
	if steering_angle > 0:
		left_count += 1
	else:
		right_count += 1

print("Left turns count: " + str(left_count))
print("Right turns count: " + str(right_count))
