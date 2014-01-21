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

#define YELLOW 1;
#define RED 2;

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
private:
	int lowerH, higherH, lowerS, higherS, lowerV, higherV;

};

//Convert ROS Image to cv image
class convertROStoCV : public Blob{
public:
	cv::Mat convertROStoCV(const sensor_msgs::ImageConstPtr& msg);
};

//Colour detection
class colourDetection : public Blob{
public:
	cv::Mat colourDetection(cv::Mat img, int colour);
	cv::Mat colourDetection(cv::Mat img, int colour, int lowerH, int higherH,
					int lowerS, int higherS, int lowerV, int higherV);

	void setWindowSettings();
	void lowerHCallback(int val, void *params);
	void higherHCallback(int val, void *params);
	void lowerSCallback(int val, void *params);
	void higherSCallback(int val, void *params);
	void lowerVCallback(int val, void *params);
	void higherVCallback(int val, void *params);
private:
	cv::Mat image;
};

#endif /* BLOB_H_ */
