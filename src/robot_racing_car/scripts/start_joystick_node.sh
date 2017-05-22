#!/bin/bash

# This is the script for launching joystick node on the master ROS machine

echo Starting ROS Joystick node:
echo

# Making sure joystick is re-emitting message for the last pressed button
rosparam set /joy_node/autorepeat_rate 1.0

rosrun joy joy_node
