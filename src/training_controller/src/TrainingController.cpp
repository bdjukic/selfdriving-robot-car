#include <ros/ros.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std::chrono;

std::ofstream training_data_file;

float steeringAngle = 0;
float throttle = 0;

void cameraCallback(const CompressedImageConstPtr& image)
{	
	cv::Mat cv_image = cv::imdecode(cv::Mat(image->data), CV_LOAD_IMAGE_COLOR);
	cv::Mat resized_cv_image;	
	cv::resize(cv_image, resized_cv_image, cv::Size(), 0.25, 0.25);
	
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	
	std::string imageFileName = "screenshot_" + std::to_string(ms.count()) + ".jpg";
	imwrite("training_data/" + imageFileName, resized_cv_image);
	
	training_data_file << imageFileName + "," + std::to_string(steeringAngle) + "," + std::to_string(throttle) + "\n";
}

void joystickCallback(const Joy::ConstPtr& joystick)
{	
	steeringAngle = joystick->axes[0];
	throttle = joystick->axes[1];
}

void sigintHandler(int sig)
{
	ROS_INFO("Saving CSV training data.");
	training_data_file.close();
	ROS_INFO("CSV traing data saved.");
	
	ros::shutdown();
}

int main(int argc, char* argv[])
{ 
	ros::init(argc, argv, "training_controller", ros::init_options::NoSigintHandler);

	ros::NodeHandle nodeHandle;
	
	signal(SIGINT, sigintHandler);
	
	ros::Subscriber joystickSubscriber = nodeHandle.subscribe<Joy>("joy", 30, &joystickCallback);
	ros::Subscriber cameraSubscriber = nodeHandle.subscribe<CompressedImage>("raspicam_node/image/compressed", 30, &cameraCallback);
	
	training_data_file.open("training_data/training_data.csv", std::ios_base::app);
	
	ROS_INFO("Training controller node started.");

	ros::spin();
	
	return 0;
}
