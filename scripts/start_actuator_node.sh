#!/bin/bash

# This is the script for launching the steering ROD node

PROJECT_FOLDER="/home/bogdan/Projects/selfdriving-robot-car/scripts"

. $PROJECT_FOLDER/setup_raspi_variables.sh

# Setting up ROS environment
source ../devel/setup.bash

echo Starting ROS Actuator controller node:
echo

rosrun actuator_controller actuator_controller
