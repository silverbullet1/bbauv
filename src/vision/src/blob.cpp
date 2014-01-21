/*
 * blob.cpp
 *  For thresholding blobs
 *  Created on: 21 Jan, 2014
 *      Author: huixian
 */

#include "blob.h"

using namespace cv;

Blob::Blob(){
	int lowerH=0, higherH=0, lowerS=0, higherS=0, lowerV=0, higherV=0;
}

Blob::~Blob(){}

/*
 * Getter and setter methods
 */
void Blob::setLowerH(int lowerH){ this->lowerH = lowerH; }
int Blob::getLowerH() { return this->lowerH; }
void Blob::setHigherH(int higherH) { this->higherH = higherH; }
int Blob::getHigherH() { return this->higherH; }
void Blob::setLowerS(int lowerS) { this->lowerS = lowerS; }
int Blob::getLowerS() { return this->lowerS;  }
void Blob::setHigherS(int higherS) { this->higherS = higherS; }
int Blob::getHigherS() { return this->higherS; }
void Blob::setLowerV(int lowerV){ this->lowerV = lowerV; }
int Blob::getLowerV() { return this->lowerV; }
void Blob::setHigherV(int higherV) { this->higherV = higherV; }
int Blob::getHigherV() { return this->higherV; }

//Converts ROS image to CV image

cv::Mat convertROStoCV::convertROStoCV(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	return cv_ptr->image;
}

/* Colour detection class */
colourDetection::colourDetection(){
	//Values are initialised as lowerH, lowerS, lowerV, higherH, higherS, higherV
	int yellow_values[6]{10, 0, 0, 79, 148, 255};
	int red_values[6]{0, 0, 100, 77, 195, 251};

	//Initialise display windows; may not be needed
	namedWindow("input");
	namedWindow("output");
	namedWindow("trackbar");
}

cv::Mat colourDetection::colourDetection(cv::Mat img, int colour){
	this->image = img;
	this->colour = colour;
	switch(colour){
	case YELLOW:
		setLowerH(yellow_values[0]);
		setHigherH(yellow_values[1]);
		setLowerS(yellow_values[2]);
		setHigherS(yellow_values[3]);
		setLowerV(yellow_values[4]);
		setHigherV(yellow_values[5]);
	break;
	case RED:
		setLowerH(red_values[0]);
		setHigherH(red_values[1]);
		setLowerS(red_values[2]);
		setHigherS(red_values[3]);
		setLowerV(red_values[4]);
		setHigherV(red_values[5]);
	break;
	}
	reDraw();
	return outImg;
}

cv::Mat colourDetection::colourDetection(cv::Mat img, int colour, int lowerH, int higherH,
					int lowerS, int higherS, int lowerV, int higherV){
	this->image = img;
	this->colour = colour;
	setLowerH(lowerH);
	setHigherH(higherH);
	setLowerS(lowerS);
	setHigherS(higherS);
	setLowerV(lowerV);
	setHigherV(higherV);
	reDraw();
	return outImg;

}

void colourDetection::reDraw(){
	cvtColor(image, outImg, CV_BGR2HSV);
	inRange(image, Scalar(lowerH,lowerS,lowerV), Scalar(higherH,higherS,higherV), outImg);
	Mat erodeEl = getStructuringElement(MORPH_RECT, cv::Size(9, 9));
	Mat dilateEl = getStructuringElement(MORPH_RECT, cv::Point(7, 7));
	erode(outImg, outImg, erodeEl, Point(-1, -1), 1);
	dilate(outImg, outImg, dilateEl, Point(-1, -1), 1);
}

void colourDetection::drawImage(){
	imshow("input", image);
	imshow("output", outImg);
}

void colourDetection::setWindowSettings(){
	createTrackbar("LowerH", "trackbar", &lowerH, 180, lowerHCallback, NULL);
    createTrackbar("UpperH", "trackbar", &upperH, 180, upperHCallback, NULL);

	createTrackbar("LowerS", "trackbar", &lowerS, 256, lowerSCallback, NULL);
    createTrackbar("UpperS", "trackbar", &upperS, 256, upperSCallback, NULL);

	createTrackbar("LowerV", "trackbar", &lowerV, 256, lowerVCallback, NULL);
    createTrackbar("UpperV", "trackbar", &upperV, 256, upperVCallback, NULL);
}

//Callback methods
void colourDetection::lowerHCallback(int val, void *params) {
	this->lowerH = val;
	reDraw();
}
void colourDetection::higherHCallback(int val, void *params){
	this->higherH = val;
	reDraw();
}
void colourDetection::lowerSCallback(int val, void *params){
	this->lowerS = val;
	reDraw();
}
void colourDetection::higherSCallback(int val, void *params){
	this->higherS = val;
	reDraw();
}
void colourDetection::lowerVCallback(int val, void *params){
	this->lowerV = val;
	reDraw();
}
void colourDetection::higherVCallback(int val, void *params){
	this->higherV = val;
	reDraw();
}


