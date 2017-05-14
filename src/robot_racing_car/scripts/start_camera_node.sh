#!/bin/bash

# This is the script for launching the Raspberry Pi camera node.

. ./setup_variables.sh

echo Starting Raspberry Pi Camera ROS node:
echo
roslaunch raspicam_node camerav2_1280x960.launch
