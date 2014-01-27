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

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <cmath>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "bbauv_msgs/compass_data.h"
#include "bbauv_msgs/controller.h"

#include "flarestates.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const int DEPTH_POINT = 1.1;
ros::Publisher movementPub;

//Utility function
double radianToDegree(double degree);

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

	//Center detection parameter
	double areaThresh;
	RectData rectData;
	cv::Size screen;
};

double radianToDegree(double degree) {
	return degree / M_PI * 180;
}

FlareDetection::FlareDetection(): it(nh), private_nh("~") {
	enabled = false;

	private_nh.param<int>("loopHz", loopRateHz, 20);
	string imageTopic; private_nh.param<std::string>("image", imageTopic, "/bottomcam/camera/image_rect_color");
	string compassTopic; private_nh.param<std::string>("compass", compassTopic, "/compass");

 	imageSub = it.subscribe(imageTopic, 1, &FlareDetection::imageCallback, this);
    compassSub = nh.subscribe(compassTopic, 1, &FlareDetection::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);

	areaThresh = 3000;
	rectData.detected = false;
	screen.width = 640;
	screen.height = 480;

	namedWindow("output");
}

FlareDetection::~FlareDetection() {
	cv::destroyWindow("output");
}

void FlareDetection::start() {
	state = boost::shared_ptr<State> (new LookForFlareState());
	enabled = true;
}

void FlareDetection::stop() {
	state = boost::shared_ptr<State>(new SurfaceState(rectData.heading));
	enabled = false;
}

void FlareDetection::compassCallback(const bbauv_msgs::compass_data& msg) {
	rectData.heading = msg.yaw;
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
	//Ignore blue stuffs
//	Mat channels[3];
//	split(inImage, channels);
//	channels[0] = Mat(channels[0].rows, channels[0].cols, channels[0].type(), Scalar::all(0));
//	Mat newImage;
//	merge(channels, 3, newImage);

}

int main(int argc, char **argv)
{
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



