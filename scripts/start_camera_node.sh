#!/bin/bash

# This is the script for launching the Raspberry Pi camera node.
# More info on the Raspberry Pi ROS camera node: https://github.com/UbiquityRobotics/raspicam_node

PROJECT_FOLDER="/home/bogdan/Projects/selfdriving-robot-car/scripts"

. $PROJECT_FOLDER/setup_raspi_variables.sh

echo Starting Raspberry Pi Camera ROS node:
echo
roslaunch raspicam_node camerav2_1280x960.launch
