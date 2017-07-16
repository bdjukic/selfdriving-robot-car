#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std_msgs;
using namespace sensor_msgs;

ros::Publisher actuatorPublisher;

void cameraCallback(const CompressedImageConstPtr& image)
{
	ROS_INFO("Callback!");
	
	cv::Mat cv_image = cv::imdecode(cv::Mat(image->data), CV_LOAD_IMAGE_COLOR);
	cv::Mat resized_cv_image;	
	cv::resize(cv_image, resized_cv_image, cv::Size(), 0.25, 0.25);
}

int main(int argc, char* argv[])
{ 
	ros::init(argc, argv, "autonomous_controller");

	ros::NodeHandle nodeHandle;
	
	ros::Subscriber cameraSubscriber = nodeHandle.subscribe<CompressedImage>("/raspicam_node/image/compressed", 1, cameraCallback);
	
	actuatorPublisher = nodeHandle.advertise<String>("/autonomous", 1);
	
	ROS_INFO("Autonomous controller node started.");

	ros::spin();
	
	return 0;
}
