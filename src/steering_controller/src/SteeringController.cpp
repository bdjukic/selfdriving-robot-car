#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32.h"
#include "PCA9685.h"

const int LEFT_ANGLE = 1;
const int RIGHT_ANGLE = -1;

const int LEFT_PULSE = 460;
const int RIGHT_PULSE = 260;

PCA9685 *actuator;

int getActuatorPulseValue(float joystickValue, int joystickMin, int joystickMax, int actuatorMin, int actuatorMax)
{
	// Linear mapping between two ranges of values
	
    int joystickRange = joystickMax - joystickMin;
    int actuatorRange = actuatorMax - actuatorMin;
    
    float joystickActuatorRatio = (float)joystickRange / (float)actuatorRange;
    int actuatorValue = (joystickValue - joystickMin) / joystickActuatorRatio + actuatorMin;

	return actuatorValue;
}

void steeringCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	int pulse = getActuatorPulseValue(joy->axes[0], RIGHT_ANGLE, LEFT_ANGLE, RIGHT_PULSE, LEFT_PULSE);
	
	actuator->setPWM(1, pulse);
	
	ROS_INFO("Steering value: [%i]", pulse);
}

int main(int argc, char* argv[])
{ 
	ros::init(argc, argv, "steering_controller");

	ros::NodeHandle nodeHandle;

	actuator = new PCA9685(1, 64);
	actuator->setPWMFreq(60);
  
	ros::Subscriber subscriber = nodeHandle.subscribe<sensor_msgs::Joy>("joy", 1, steeringCallback);
	
	ROS_INFO("Steering controller ROS node started.");

	ros::spin();
	
	return 0;
}
