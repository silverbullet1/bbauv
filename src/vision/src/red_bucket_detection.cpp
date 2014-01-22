/* 
	red_bucket_detection.cpp
	Date created: Jan 2014
	Author: Jason Poh
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 	cv_bridge::CvImagePtr cv_ptr;	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}catch(cv_bridge::Exception& e){
		cv_bridge::CvImagePtr cv_ptr;
	}
}

int main(int argc, char **argv)
{
	/*
		Initalization
	*/
	ros::init(argc, argv, "red_bucket_detection");
	ros::NodeHandle node;
  	image_transport::ImageTransport imgTransport(node);
  	image_transport::Subscriber image_sub;
  	string bottom_camera_topic_name = "/camera/image_raw";

  	/*
		Get params from ROS Param Server
	*/
	if (node.hasParam("/red_bucket_detection/bottom_camera_topic"))
	{
		if(node.getParam("/red_bucket_detection/bottom_camera_topic", bottom_camera_topic_name))
		{
			ROS_INFO("Subscribing to bottom camera topic @ %s", bottom_camera_topic_name.c_str());
		}
	}

  	image_sub = imgTransport.subscribe(bottom_camera_topic_name, 1, imageCallback);

  	ros::spin();
	return 0;
}
