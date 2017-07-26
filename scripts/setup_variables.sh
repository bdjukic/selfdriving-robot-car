#!/bin/bash

echo
echo Setting up ROS environment variables:
echo

# IP address for the robot car
export ROS_IP="192.168.0.17"

# IP address for the machine which does training and inference
export ROS_MASTER_URI="http://192.168.0.10:11311"

echo ROS_IP=$ROS_IP
echo ROS_MASTER_URI=$ROS_MASTER_URI
echo
