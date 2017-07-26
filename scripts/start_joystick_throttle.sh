#!/bin/bash

# This is the script for starting joystick throttle

PROJECT_FOLDER="/home/bogdan/Projects/selfdriving-robot-car/scripts"

. $PROJECT_FOLDER/setup_rosmaster_variables.sh

echo Starting ROS Joystick topic throttle:
echo

rosrun topic_tools throttle messages joy 100.0
