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

//Utility function
double radianToDegree(double degree);

//Initialize global variables
//const int endTime = 0;
const int DEPTH_POINT = 1.1;
const double secondsToRun = 2.25 * 60;
const double x_strip_threshold = 0.2;
ros::Publisher movementPub;

//Main class for linefollower node
class LineFollower
{
public:
	LineFollower();
	~LineFollower();

	int loopRateHz;

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

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;
	//Subscribers to respective topic
	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	//ros::Publisher movementPub;
	image_transport::ImageTransport it;

	//Center detection parameter
	double thVal;
	double areaThresh;
	RectData rectData;
	cv::Size screen;

	//States
	boost::shared_ptr<State> state;
};

double radianToDegree(double degree) {
	return degree / M_PI * 180;
}

LineFollower::LineFollower() : it(nh), private_nh("~")
{
	enabled = false;

	private_nh.param<int>("loopHz", loopRateHz, 20);
	string imageTopic; private_nh.param<string>("image", imageTopic, "/bottomcam/camera/image_raw");
	string compassTopic; private_nh.param<string>("compass", compassTopic, "/os5000_data");

 	imageSub = it.subscribe("/camera2/camera/image_raw", 1, &LineFollower::bottomCamCallback, this);
    compassSub = nh.subscribe("/AHRS8_data_e", 1, &LineFollower::compassCallback, this);
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
	boost::shared_ptr<State> nextState(new StraightLineState());
	state = boost::shared_ptr<State>(new DiveState(0.2, nextState));
	enabled = true;
}

void LineFollower::stop() {
	state = boost::shared_ptr<State>(new SurfaceState(rectData.heading));
	enabled = false;
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

//	Mat roiImg;
//	Rect roi(0, 190, 640, 100);
//	greyImg(roi).copyTo(roiImg);

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
		state = state->gotFrame(inImage, rectData);
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
	LineFollower linefollower;
	linefollower.start();
	ROS_INFO("Initialised LineFollower...");

	signal(SIGINT, quit);

	ros::Rate loop_rate(linefollower.loopRateHz);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return (0);
}
