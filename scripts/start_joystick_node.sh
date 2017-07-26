#!/bin/bash

# This is the script for launching joystick node on the master ROS machine

PROJECT_FOLDER="/home/bogdan/Projects/selfdriving-robot-car/scripts"

. $PROJECT_FOLDER/setup_rosmaster_variables.sh

echo Starting ROS Joystick node:
echo

# Making sure joystick is re-emitting message for the last pressed button
rosparam set /joy_node/autorepeat_rate 1.0

rosrun joy joy_node
