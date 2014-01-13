/*
 * BBSonar.h
 *
 *  Created on: Dec 14, 2013
 *      Author: freax
 */

#ifndef BBSONAR_H_
#define BBSONAR_H_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "bvt_sdk.h"
#include "Config.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

typedef unsigned int uInt;
typedef unsigned short uShort;

class BBSonar {
public:
//	bvtsdk params
	BVTSonar son, fson;
	BVTHead head;
	BVTPing ping;
	BVTRangeData rangeData;
	BVTMagImage magImg;
	BVTColorImage colorImg;
	BVTColorMapper colorMap;

	int retVal;

//	head params
	float startRange;
	float stopRange;
	int fluidType;
	int soundSpeed;
	float analogGain;
	float tvGain;
	int pingCount;
	int imgFilterFlags;

//	io params
	ofstream rangeStream;
	ofstream bearingStream;
	ofstream intensityOutStream;
	ifstream intensityInStream;

//	image pointers
	IplImage* grayImg;
//	IplImage* iColorImg;
//	IplImage* iFiltImg;
//	IplImage* iEdgeImg;
//	IplImage* iThreshImg;
//	IplImage* iLabeledImg;
//	IplImage* iMorphImg;
	IplImage* grayImg8b;
	IplImage* grayImg16b;

	BBSonar();
	virtual ~BBSonar();

	void getSetHeadParams();
	void setHeadParams();

	int initSonar();
	int retrievePing();
	int processPing();
	int processImage();
	int processObjectPixels(CvPoint, CvPoint);
	int saveRangeBearingData();
	int writeIntensities(const IplImage* grayImg);
	int readIntensities();
	double getGlobalThreshold(CvArr* gImg);

	int writeSonarLog();
	int readSonarLog();

	int disableTransmission();

private:
	int imgWidth;
	int imgHeight;
	int imgWidthStep;

};

#endif /* BBSONAR_H_ */
