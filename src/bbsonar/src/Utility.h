/*
 * Utility.h
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>
#include <string>

#include "bvt_sdk.h"
#include "Config.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include <bbauv_msgs/sonarData.h>
#include <bbauv_msgs/sonarDataVector.h>

using namespace std;
using namespace cv;

typedef unsigned short uShort;
typedef unsigned int uInt;

class Utility {
public:
	BVTSonar son, fson;
	BVTHead head;
	BVTPing ping;
	BVTRangeData rangeData;
	BVTMagImage magImg;
	BVTColorImage colorImg;
	BVTColorMapper colorMap;

	cv::Mat grayImg;

	//	head params
	float startRange;
	float stopRange;
	int fluidType;
	int soundSpeed;
	float analogGain;
	float tvGain;
	int pingCount;
	int rangeValCount;

	uShort *imgBuffer;
	int retVal;

	cv::Mat matImg;

	vector<vector<Point> > savedContours;
	vector<Point> savedPoints;

	Utility();
	virtual ~Utility();

	int initSonar();
	int setHeadParams();
	int processImage();
	int intensitiesToImage();
	int writeIntensities();
	int drawHistogram();

	inline const std::string currentDateTime();
	void delay(long delay);

	double getGlobalThreshold(cv::Mat gImg);
	void myAdaptiveThreshold(cv::Mat gImg, double maxValue, int method, int type, int blockSize, double delta);
	
	bool getRangeBearing();

	int imgWidth;
	int imgHeight;
	int imgWidthStep;

  bbauv_msgs::sonarData singlePoint;
	bbauv_msgs::sonarDataVector sonarMsg;

private:
};

#endif /* UTILITY_H_ */
