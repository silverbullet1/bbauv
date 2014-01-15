/* 
	linefollower.cpp
	Date created: 10 Jan 2014
	Author: Lynnette & Thien
*/

#include <ros/ros.h>
#include "bbauv_msgs/compass_data.h"
#include "bbauv_msgs/controller.h"
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "linefollowingstates.h"

using namespace cv;

//static double heading = 0.0;

//Utility functions
double normHeading(double heading);
double radianToDegree(double degree);

class LineFollower
{
public:
	LineFollower();
	~LineFollower();

	void start();
	void stop();

	void compassCallback(const bbauv_msgs::compass_data& msg);
	void publishMovement(const bbauv_msgs::controller& movement);
	double normHeading(double heading);	
	void bottomCamCallback(const sensor_msgs::ImageConstPtr& msg);

	//Fill rectData structure with necessary data
	void prepareBlackLineParams(Mat inImage);
private:
	bool enabled;

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
	RectData rectData;
	cv::Size screen;

	//States
	boost::shared_ptr<State> state;
};

//Function to normalise heading in degrees
double normHeading(double heading)
{
	if (heading > 360.0) { return heading - 360.0; }
	else if (heading < 0.0) { return heading + 360.0; }
	else { return heading; }
}

double radianToDegree(double degree) {
	return degree / M_PI * 180;
}

LineFollower::LineFollower() : it(nh)
{
	enabled = false;

 	imageSub = it.subscribe("/bumblebee/bottomCam", 1, &LineFollower::bottomCamCallback, this);
    compassSub = nh.subscribe("/compass", 1, &LineFollower::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);

	thVal = 80;
	areaThresh = 5000;
	rectData.detected = false;
	screen.width = 640;
	screen.height = 480;

	namedWindow("test");
}

LineFollower::~LineFollower() {
	cv::destroyWindow("test");
}

void LineFollower::start() {
	enabled = true;
}

void LineFollower::stop() {
	enabled = false;
}

void LineFollower::publishMovement(const bbauv_msgs::controller& movement){
 	movementPub.publish(movement);
}

void LineFollower::compassCallback(const bbauv_msgs::compass_data& msg){
	rectData.heading = msg.yaw;
}

// Convert ROS image to CV image
void LineFollower::bottomCamCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	prepareBlackLineParams(cv_ptr->image);
}

void LineFollower::prepareBlackLineParams(Mat inImage) {
	//Ignore blue stuffs
	Mat channels[3];
	split(inImage, channels);
	channels[0] = Mat(channels[0].rows, channels[0].cols, channels[0].type(), Scalar::all(0));
	Mat newImage;
	merge(channels, 3, newImage);

	Mat greyImg;
	cvtColor(newImage, greyImg, CV_BGR2GRAY);
	resize(greyImg, greyImg, screen);

	//Thresholding and noise removal
	GaussianBlur(greyImg, greyImg, Size(5, 5), 0, 0);
	threshold(greyImg, greyImg, thVal, 255, THRESH_BINARY_INV);
	Mat erodeEl = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateEl = getStructuringElement(MORPH_RECT, Point(5, 5));
	erode(greyImg, greyImg, erodeEl);
	dilate(greyImg, greyImg, dilateEl);

	//Testing Mat
	cv::Mat out = inImage.clone();
	resize(out, out, Size(640, 480));

	//Find x-center
	cv::vector< cv::vector<Point> > contours;
	cv::vector<cv::Vec4i> hierachy;

	findContours(greyImg, contours, hierachy,
				 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	double max_area = 0;
	Point2f center_max;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > areaThresh && area > max_area) {
			//Find the center using moments
			Moments mu;
			mu = moments(contours[i], false);
			center_max.x = mu.m10/mu.m00;
			center_max.y = mu.m01/mu.m00;
			max_area = area;

			//Find the blackline bounding rect
			rectData.maxRect = minAreaRect(contours[i]);

		}
	}

	if (max_area > 0) {
		rectData.detected = true;
		rectData.center = center_max;

		//Find the blackline heading
		Point2f points[4];
		rectData.maxRect.points(points);
		Point2f edge1 = points[1] - points[0];
		Point2f edge2 = points[2] - points[1];
		//Choose the verticle edge
		if (norm(edge1) > norm(edge2)) {
			rectData.angle = radianToDegree(atan(edge1.x/edge1.y));
		} else {
			rectData.angle = radianToDegree(atan(edge2.x/edge2.y));
		}
		//Chose angle to turn if horizontal
		if (rectData.angle == 90) {
			if (rectData.center.x > (screen.width / 2)) {
				rectData.angle = -90;
			}
		} else if (rectData.angle == -90) {
			if (rectData.center.x < (screen.width) / 2) {
				rectData.angle = 90;
			}
		}

		//ROS_INFO("area: %lf angle: %lf", max_area, rectData.angle);

		//Testing
		circle(out, rectData.center, 5, Scalar(0, 255, 0));
		for (int i = 0; i < 4; i++) {
			Point2i pt1(int(points[i].x), int(points[i].y));
			Point2i pt2(int(points[(i+1)%4].x), int(points[(i+1)%4].y));
			cv::line(out, pt1, pt2, Scalar(0, 0, 255));
		}
		imshow("test", out);
		waitKey(5);
	} else {
		rectData.detected = false;
	}

	if (enabled)
		state->gotFrame(inImage, rectData);
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
//	int loopRate = 20;
//	ros::Rate loop_rate(loopRate);
	LineFollower linefollower;
	ROS_INFO("Initialsing LineFollower...");

	signal(SIGINT, quit);

	ros::spin();
	//loop_rate.sleep();

	return (0);
}
