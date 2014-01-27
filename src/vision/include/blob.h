/*
 * blob.h
 *
 *  Created on: 21 Jan, 2014
 *      Author: huixian
 */

#ifndef BLOB_H_
#define BLOB_H_

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <stdlib.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define YELLOW 1
#define RED 2

//Structure for drawing bounding box
struct RectData {
	bool detected;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Global callback functions
void lowerHCallback(int val, void *params);
void higherHCallback(int val, void *params);
void lowerSCallback(int val, void *params);
void higherSCallback(int val, void *params);
void lowerVCallback(int val, void *params);
void higherVCallback(int val, void *params);

//We call this class blob!
class Blob {
public:
	Blob() {}
	virtual ~Blob() {}

	void setLowerH(int lowerH);
	int getLowerH();
	void setHigherH(int higherH);
	int getHigherH();
	void setLowerS(int lowerS);
	int getLowerS();
	void setHigherS(int higherS);
	int getHigherS();
	void setLowerV(int lowerV);
	int getLowerV();
	void setHigherV(int higherV);
	int getHigherV();

	static int lowerH, higherH, lowerS, higherS, lowerV, higherV;

private:
	RectData rectData;
};

//Convert ROS Image to cv image
class convertROStoCV : public Blob{
public:
	cv::Mat convertROStoCV(const sensor_msgs::ImageConstPtr& msg);
};

//Colour detection
class colourDetection : public Blob{
public:
	colourDetection();
	cv::Mat colourDetection(cv::Mat img, int colour);
	cv::Mat colourDetection(cv::Mat img, int colour, int lowerH, int higherH,
					int lowerS, int higherS, int lowerV, int higherV);

	void reDraw();
	void drawImage();
	void setWindowSettings();
	cv::Mat drawBoundingBox(cv::Mat img);
	void findBoundingBox(cv::Mat img);

private:
	cv::Mat image;
	cv::Mat outImg;
	int colour;
	//Values are initialised as lowerH, lowerS, lowerV, higherH, higherS, higherV
	int yellow_values[6];
	int red_values[6];
	//For bounding box
	double max_area, areaThresh;

};

#endif /* BLOB_H_ */
