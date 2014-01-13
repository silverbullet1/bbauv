/* 
	linefollower.cpp
	Date created: 10 Jan 2014
	Author: Lynnette
*/

#include <ros/ros.h>
#include "bbauv_msgs/compass_data.h"
#include "bbauv_msgs/controller.h"
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "linefollowingstates.h"

using namespace cv;

static double heading = 0.0;
static bool enabled = false;

class LineFollower
{
public:
	LineFollower();

	void compassCallback(const bbauv_msgs::compass_data& msg);
	void publishMovement(const bbauv_msgs::controller& movement);
	double normHeading(double heading);	
	void bottomCamCallback(const sensor_msgs::ImageConstPtr& msg);

	Point2f blackLineXCenter(Mat inImage);
private: 

	static const int endTime = 0;
	static const int DEPTH_POINT = 1.1;
	static const double secondsToRun = 2.25 * 60;

	ros::NodeHandle nh;
	//Subscribers to respective topic
	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	ros::Publisher movementPub;
	image_transport::ImageTransport it;

	//Center detection parameter
	double thVal;
	double areaThresh;
};

LineFollower::LineFollower() : it(nh)
{
 	imageSub = it.subscribe("/bumblebee/bottomcam", 1, &LineFollower::bottomCamCallback, this);
    compassSub = nh.subscribe("/compass", 1, &LineFollower::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);

	thVal = 30;
	areaThresh = 2000;
}

int kfd = 0;
struct termios cooked;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "linefollower");
	int loopRate = 20;
	ros::Rate loop_rate(loopRate);
	LineFollower linefollower;
	ROS_INFO("Initialsing LineFollower...");

	signal(SIGINT, quit);

	ros::spinOnce();
	loop_rate.sleep();

	return (0);
}

//Function to normalise heading in degrees
double LineFollower::normHeading(double heading)
{
	if (heading > 360.0) { return heading - 360.0; }
	else if (heading < 0.0) { return heading + 360.0; }
	else { return heading; }
}

void LineFollower::publishMovement(const bbauv_msgs::controller& movement){
 	movementPub.publish(movement);
}

void LineFollower::compassCallback(const bbauv_msgs::compass_data& msg){
	heading = msg.yaw;
}

// Convert ROS image to CV image
void LineFollower::bottomCamCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	// Do something with cv_ptr
	std::cout << blackLineXCenter(cv_ptr->image) << std::endl;
}

Point2f LineFollower::blackLineXCenter(Mat inImage) {
	Mat greyImg;
	cvtColor(inImage, greyImg, CV_BGR2GRAY);
	resize(greyImg, greyImg, Size(640, 480));
	//ROI
	Mat roiImg;
	Rect roi(0, 190, 640, 100);
	greyImg(roi).copyTo(roiImg);

	//Thresholding and noise removal
	GaussianBlur(roiImg, roiImg, Size(5, 5), 0, 0);
	threshold(roiImg, roiImg, thVal, 255, THRESH_BINARY_INV);
	Mat erodeEl = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateEl = getStructuringElement(MORPH_RECT, Point(5, 5));
	erode(roiImg, roiImg, erodeEl);
	dilate(roiImg, roiImg, dilateEl);

	//Find x-center
	cv::Mat out = inImage.clone();
	resize(out, out, Size(640, 480));
	cv::vector< cv::vector<Point> > contours;
	cv::vector<cv::Vec4i> hierachy;

	findContours(roiImg, contours, hierachy,
				 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (size_t i = 0; i < contours.size(); i++) {
		float area = contourArea(contours[i]);
		if (area > areaThresh) {
			Moments mu;
			mu = moments(contours[i], false);
			Point2f center(mu.m10/mu.m00, 240);
			return center;
		}
	}

	return Point2f(-1, -1);
}
