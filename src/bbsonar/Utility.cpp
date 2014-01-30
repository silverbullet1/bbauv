/*
 * Utility.cpp
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#include "Utility.h"
using namespace cv;

Utility::Utility() : son(NULL), fson(NULL), head(NULL), ping(NULL), rangeData(NULL), magImg(NULL), colorImg(NULL),
		colorMap(NULL), grayImg(NULL), grayImg8b(NULL), grayImg16b(NULL), imgBuffer(NULL) {

	retVal = startRange = stopRange = fluidType = soundSpeed = analogGain = tvGain =
	pingCount = imgWidth = imgHeight = imgWidthStep = rangeValCount = 0;
}

Utility::~Utility() {
	if (imgBuffer != NULL) imgBuffer = NULL;
	if (son != NULL) BVTSonar_Destroy(son);
	if (ping != NULL) BVTPing_Destroy(ping);
}

int Utility::initSonar() {
	son = BVTSonar_Create();
	retVal = BVTSonar_Open(son, "NET", "192.168.1.45");
//	retVal = BVTSonar_Open(son, "FILE", "data/salmon_small.son");

	if((retVal = BVTSonar_GetHead(son, HEAD_NUM, &head)) != 0) {
		cout << "error retrieving head, exiting now: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
		cout << "error retrieving ping: "  << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	return 0;
}

int Utility::setHeadParams() {
	startRange = BVTHead_GetStartRange(head);
//	stopRange = BVTHead_GetStopRange(head);
	fluidType = BVTHead_GetFluidType(head);
	soundSpeed = BVTHead_GetSoundSpeed(head);
	analogGain = BVTHead_GetGainAdjustment(head);
	tvGain = BVTHead_GetTVGSlope(head);
	pingCount = BVTHead_GetPingCount(head);
	stopRange = MAX_RANGE;

	cout << "(Start, Stop) range:\t" << BVTHead_GetStartRange(head) << "\t" << BVTHead_GetStopRange(head) << endl;

	//sets
	if((retVal = BVTHead_SetRange(head, startRange, stopRange)) != 0)
			cout << "error setting range" << endl;

	if((retVal = BVTHead_SetImageRes(head, RES_TYPE)) != 0)
		cout << "error setting image resolution" << endl;

	if((retVal = BVTHead_SetImageType(head, IMAGE_TYPE)) != 0)
		cout << "error setting image type" << endl;

	if((retVal = BVTHead_SetFluidType(head, fluidType)) != 0)
		cout << "error setting fluid type" << endl;

	if((retVal = BVTHead_SetSoundSpeed(head, soundSpeed)) != 0)
		cout << "error setting sound speed" << endl;

	if((retVal = BVTHead_SetGainAdjustment(head, analogGain)) != 0)
		cout << "error setting analog gain" << endl;

	if((retVal = BVTHead_SetTVGSlope(head, tvGain)) != 0)
		cout << "error setting TV Gain" << endl;

	return 0;
}

int Utility::writeIntensities() {
	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		cout << "error retrieving ping: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}

	BVTMagImage_SavePGM(magImg, GRAYSCALE_MAG_FILE.c_str());
	if((colorMap = BVTColorMapper_Create()) == NULL) {
		cout << "error creating color mapper" << endl;
		return -1;
	}
	if((retVal = BVTColorMapper_Load(colorMap, COLOR_MAPPER_PATH.c_str())) != 0) {
		cout << "error retrieving color map: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTColorMapper_MapImage(colorMap, magImg, &colorImg)) != 0) {
		cout << "error mapping to color image: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	BVTColorImage_SavePPM(colorImg, COLOR_IMAGE_FILE.c_str());
	cout << "magImg size: " << "height: " << BVTMagImage_GetHeight(magImg) << "  width: " << BVTMagImage_GetWidth(magImg) <<  endl;

	imgBuffer = BVTMagImage_GetBits(magImg);
	if (imgBuffer == NULL) cout << "imgBuf null" << endl;
	matImg = Mat(BVTMagImage_GetHeight(magImg), BVTMagImage_GetWidth(magImg), CV_16UC1, imgBuffer);
//	cout  << "matImg depth, channel: " << matImg.depth() << " " << matImg.channels() << endl;

	imwrite("newIntensities.png", matImg);
	cout << "matImg size: " << "height: " << matImg.rows << "  width: " << matImg.cols << endl;

	ofstream iOut("newIntensities.txt");
	for(int row=0; row < matImg.rows; ++row) {
		for(int col=0; col < matImg.cols; ++col) {
			cv::Scalar intensity = matImg.at<uchar>(col, row);
			iOut << (uchar)(intensity.val[0] > GRAYSCALE_THRESH ? intensity.val[0] : 0) ;
			iOut << " " ;
		}
		iOut << endl;
	}
	iOut.close();

//	backup data storage in xml format
	cv::FileStorage storage("store.yml", cv::FileStorage::WRITE);
	storage << "mat" << matImg;
	storage.release();

	imshow("matImg", matImg);
	cv::waitKey(0);

	if (NULL != magImg) BVTMagImage_Destroy(magImg);
	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);

	return 0;
}

int Utility::processImage() {
//	ifstream intensityIn;

//	reading the image from the stored xml file
//	cv::Mat grayImg = Mat::zeros(BVTMagImage_GetHeight(magImg), BVTMagImage_GetWidth(magImg), CV_16UC1);
//	cv::FileStorage storage("store.yml", cv::FileStorage::READ);
//	storage["mat"] >> grayImg;
//	storage.release();

//	hardcoded read : beware of the image's path
//	cv::Mat grayImg = Mat::zeros(316, 250, CV_16UC1);
	cv::Mat grayImg = imread("newIntensities.png", 0);

//	all processing stuffs with the saved grayscale image
//	cv::Mat grayImg = matImg.clone();
	cv::Mat smoothImg, threshImg, edgedImg, morphOImg, morphCImg, labelledImg;

//	initializing the image matrices with zeros
	smoothImg = Mat::zeros(grayImg.rows, grayImg.cols, CV_16UC1);
	threshImg = Mat::zeros(grayImg.rows, grayImg.cols, CV_16UC1);
	edgedImg = Mat::zeros(grayImg.rows, grayImg.cols, CV_16UC1);
	morphOImg = Mat::zeros(grayImg.rows, grayImg.cols, CV_16UC1);
	morphCImg = Mat::zeros(grayImg.rows, grayImg.cols, CV_16UC1);

//	smoothened
	cv::medianBlur(grayImg, smoothImg, 3);

//	thresholded : hardcoded for testing
//	double threshVal = getGlobalThreshold(grayImg);
	double threshVal = 150;
//	cout << "global threshold value: " << threshVal << endl;
	cv::threshold(smoothImg, threshImg, threshVal, 255, CV_THRESH_BINARY);

//	morphology : opening and closing
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
	morphologyEx(threshImg, morphOImg, CV_MOP_OPEN, element);
	morphologyEx(threshImg, morphCImg, CV_MOP_CLOSE, element);

//	applying canny filter for detecting edges
	Canny(morphOImg, edgedImg, 1.0, 3.0, 3);

//	labelled image
//	RNG rng;
//	vector<vector<Point> > contours;
//	vector<Vec4i> hierarchy;
//	findContours(edgedImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//	labelledImg = Mat::zeros(edgedImg.size(), CV_16UC1);
//	for(int i = 0; i< contours.size(); i++) {
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		drawContours(labelledImg, contours, i, color, 2, 8, hierarchy, 0, Point());
//	}

	imshow("grayImg", grayImg);
	imshow("smoothened", smoothImg);
	imshow("thresholded", threshImg);
	imshow("morphOpened", morphOImg);
	imshow("morphClosed", morphCImg);
	imshow("edged", edgedImg);
//	imshow("labelled", labelledImg);

	cvWaitKey(0);

	return 0;
}

double Utility::getGlobalThreshold(cv::Mat gImg) {
	cv::Mat g, ginv;
	bool done;
	double T, Tnext;
	double min_val, max_val;

	cv::minMaxLoc(gImg, &min_val, &max_val, NULL, NULL, NULL);
	cout << "Min val: " << min_val << "\tMax val: " << max_val << endl;

	T = 0.5 * (min_val + max_val);
	done = false;

	while(!done) {
		cv::threshold(gImg, g, T, 255, CV_THRESH_BINARY);
		cv::bitwise_not(g, ginv);
		Tnext = 0.5 * (cv::mean(gImg, g).val[0] + cv::mean(gImg, ginv).val[0]);
		done = fabs(T-Tnext) < 5;
		T = Tnext;
	}
	return T;
}
