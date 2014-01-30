/*
 * Utility.h
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "bvt_sdk.h"
#include "Config.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

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

	IplImage* grayImg;
	IplImage* grayImg8b;
	IplImage* grayImg16b;

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

	Utility();
	virtual ~Utility();

	int initSonar();
	int setHeadParams();
	int processImage();
	int intensitiesToImage();
	int writeIntensities();

	double getGlobalThreshold(cv::Mat gImg);

private:
	int imgWidth;
	int imgHeight;
	int imgWidthStep;
};

#endif /* UTILITY_H_ */
