# Setup instructions 

## ROS Master machine setup: 

1. Install Ubuntu 16.04
2. Install ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu  
3. Install ROS joystick package: `sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers`
4. Install Catkin build tools: http://catkin-tools.readthedocs.io/en/latest/installing.html

## Raspberry Pi setup:

1. Install latest Ubuntu MATE: https://ubuntu-mate.org/raspberry-pi/
2. Install ROS package for Raspberry Pi camera: http://ubiquityrobotics.com/blog/2016/01/24/Raspberry-Pi-Cam.html
3. Install Catkin build tools: http://catkin-tools.readthedocs.io/en/latest/installing.html

## Building the workspace:

Run `catkin build` in the root of the project.

## Setting up environment variables

Set up ROS Master node and Raspberry Pi IP addresses in `scripts/setup_variables.sh` 
