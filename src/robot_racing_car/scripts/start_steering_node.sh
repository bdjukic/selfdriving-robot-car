#!/bin/bash

# This is the script for launching the steering ROD node

PROJECT_FOLDER="/home/bogdan/Projects/selfdriving-robot-car/src/robot_racing_car/scripts"

. $PROJECT_FOLDER/setup_variables.sh

echo Starting ROS steering node:
echo
python ../nodes/steering_controller.py
