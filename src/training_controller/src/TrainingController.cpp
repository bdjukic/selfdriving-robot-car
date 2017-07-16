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

void callback(const Joy::ConstPtr& joystick, const CompressedImageConstPtr& image)
{	
	float steeringAngle = joystick->axes[0];
	float throttle = joystick->axes[1];
	
	cv::Mat cv_image = cv::imdecode(cv::Mat(image->data), CV_LOAD_IMAGE_COLOR);
	cv::Mat resized_cv_image;	
	cv::resize(cv_image, resized_cv_image, cv::Size(), 0.25, 0.25);
	
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	
	std::string imageFileName = "training_data/screenshot_" + std::to_string(ms.count()) + ".jpg";
	imwrite(imageFileName, resized_cv_image);
	
	training_data_file << imageFileName + "," + std::to_string(steeringAngle) + "," + std::to_string(throttle) + "\n";
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
	
	message_filters::Subscriber<Joy> joystickSubscriber(nodeHandle, "/joy_throttle", 10);
	message_filters::Subscriber<CompressedImage> cameraSubscriber(nodeHandle, "/raspicam_node/image/compressed", 10);
	
	typedef sync_policies::ApproximateTime<Joy, CompressedImage> SyncPolicy;
	
	Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), joystickSubscriber, cameraSubscriber);
	synchronizer.registerCallback(boost::bind(&callback, _1, _2));
	
	training_data_file.open("training_data/training_data.csv", std::ios_base::app);
	
	ROS_INFO("Training controller node started.");

	ros::spin();
	
	return 0;
}
