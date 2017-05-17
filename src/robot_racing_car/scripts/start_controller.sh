#!/bin/bash

echo
echo Setting up ROS environment variables:
echo

export ROS_IP="192.168.0.20"

echo ROS_IP=$ROS_IP
echo

echo Starting controller...
echo

python ../nodes/robot_car_controller.py
