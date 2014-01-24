/*
 * flaredetection.cpp
 *
 *  Created on: 24 Jan, 2014
 *      Author: ndt
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/controller.h>
#include "flarestates.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const int DEPTH_POINT = 1.1;
ros::Publisher movementPub;

class FlareDetection
{
public:
	FlareDetection();
	~FlareDetection();

	int loopRateHz;

	void start();
	void stop();

	void compassCallback(const bbauv_msgs::compass_data& msg);
	double normHeading(double heading);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	//Fill rectData structure with necessary data
	void prepareFlareParams(Mat inImage);
private:
	bool enabled;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	image_transport::ImageTransport it;

	//States
	boost::shared_ptr<State> state;
};

FlareDetection::FlareDetection(): it(nh) {
	enabled = false;

	private_nh.param<int>("loopHz", loopRateHz, 20);
	string imageTopic; private_nh.param<std::string>("image", imageTopic, "/bottomcam/camera/image_rect_color");
	string compassTopic; private_nh.param<std::string>("compass", compassTopic, "/compass");

 	imageSub = it.subscribe(imageTopic, 1, &FlareDetection::imageCallback, this);
    compassSub = nh.subscribe(compassTopic, 1, &FlareDetection::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);
}

FlareDetection::~FlareDetection() {

}

void FlareDetection::start() {
	enabled = true;
}

void FlareDetection::stop() {
	enabled = false;
}

void FlareDetection::compassCallback(const bbauv_msgs::compass_data& msg) {

}

void FlareDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}catch(cv_bridge::Exception& e){
		cv_bridge::CvImagePtr cv_ptr;
	}

	prepareFlareParams(cv_ptr->image);
}

void FlareDetection::prepareFlareParams(Mat image) {
	//Do image processing here

	//Call state

}

int main(int argc, char **argv)
{
	/*
		Initalization
	*/
	ros::init(argc, argv, "flare_detection");

	FlareDetection flareDetector;
	flareDetector.start();
	ROS_INFO("Initialised Flare Detection...");

	ros::Rate loop_rate(flareDetector.loopRateHz);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  	ros::spin();
	return 0;
}



