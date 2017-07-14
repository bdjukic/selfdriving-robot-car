#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "std_msgs/Float32.h"
#include "PCA9685.h"

// Steering constants
const int MAX_LEFT_ANGLE = 1;
const int MIN_RIGHT_ANGLE = -1;

const int MAX_STEERING_PULSE = 460;
const int MIN_STEERING_PULSE = 260;

const int STEERING_ACTUATOR_CHANNEL = 2;

// Throttle constants
const int MIN_THROTTLE = -1;
const int MAX_THROTTLE = 1;

const int ZERO_PULSE = 370;
const int MIN_THROTTLE_PULSE = 220;
const int MAX_THROTTLE_PULSE = 500;

const int THROTTLE_ACTUATOR_CHANNEL = 1;

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
	float steeringValue = joy->axes[0];
	float throttleValue = joy->axes[1];
	
	int steeringPulse = getActuatorPulseValue(steeringValue, MIN_RIGHT_ANGLE, MAX_LEFT_ANGLE, MIN_STEERING_PULSE, MAX_STEERING_PULSE);
    	int throttlePulse = getActuatorPulseValue(throttleValue, MIN_THROTTLE, MAX_THROTTLE, MIN_THROTTLE_PULSE, MAX_THROTTLE_PULSE);
	
	actuator->setPWM(STEERING_ACTUATOR_CHANNEL, steeringPulse);
	actuator->setPWM(THROTTLE_ACTUATOR_CHANNEL, throttlePulse);
	
	ROS_INFO("Steering value: [%i]", steeringPulse);
	ROS_INFO("Throttle value: [%i]", throttlePulse);
}

int main(int argc, char* argv[])
{ 
	ros::init(argc, argv, "actuator_controller");

	ros::NodeHandle nodeHandle;

	actuator = new PCA9685(1, 64);
	actuator->setPWMFreq(60);
  
	ros::Subscriber subscriber = nodeHandle.subscribe<sensor_msgs::Joy>("joy", 1, steeringCallback);
	
	ROS_INFO("ROS Actuator controller node started.");

	ros::spin();
	
	return 0;
}
