#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "PCA9685.h"

// General constans
const int I2C_PORT = 1;
const int I2C_ADDRESS = 64;
const int PCA9685_FREQUENCY = 60;

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

using namespace std_msgs;
using namespace sensor_msgs;
using namespace std;

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

void drive(float steeringAngle, float throttle)
{
     int steeringPulse = getActuatorPulseValue(steeringAngle, MIN_RIGHT_ANGLE, MAX_LEFT_ANGLE, MIN_STEERING_PULSE, MAX_STEERING_PULSE);
     int throttlePulse = getActuatorPulseValue(throttle, MIN_THROTTLE, MAX_THROTTLE, MIN_THROTTLE_PULSE, MAX_THROTTLE_PULSE);
	
     actuator->setPWM(STEERING_ACTUATOR_CHANNEL, steeringPulse);
     actuator->setPWM(THROTTLE_ACTUATOR_CHANNEL, throttlePulse);
}

void steeringCallback(const Joy::ConstPtr& joy)
{
     float steeringAngle = joy->axes[0];
     float throttle = joy->axes[1];
	
     drive(steeringAngle, throttle / 4.0);
}

vector<string> splitString(const string &s, char delimiter) {
    stringstream ss(s);
    string item;
    vector<string> tokens;
    
    while (getline(ss, item, delimiter)) {
        tokens.push_back(item);
    }
    
    return tokens;
}

void autonomousCallback(const String::ConstPtr& command)
{	
	vector<string> parsedCommand = splitString(command->data, ':');
	
	float steeringAngle = atof(parsedCommand[0].c_str());
	float throttle = atof(parsedCommand[1].c_str());
	
	drive(steeringAngle, throttle);
}

int main(int argc, char* argv[])
{ 
	ros::init(argc, argv, "actuator_controller");

	ros::NodeHandle nodeHandle;

	actuator = new PCA9685(I2C_PORT, I2C_ADDRESS);
	actuator->setPWMFreq(PCA9685_FREQUENCY);
  
	ros::Subscriber steeringSubscriber = nodeHandle.subscribe<Joy>("joy", 1, steeringCallback);
	ros::Subscriber autonomousSubscriber = nodeHandle.subscribe<String>("autopilot", 1, autonomousCallback);
	
	ROS_INFO("ROS Actuator controller node started.");

	ros::spin();
	
	return 0;
}
