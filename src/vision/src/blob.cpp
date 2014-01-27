/*
 * blob.cpp
 *  For thresholding blobs
 *  Created on: 21 Jan, 2014
 *      Author: huixian
 */

#include "blob.h"

using namespace cv;

int Blob::lowerH=0;
static int Blob::higherH=0;
static int Blob::lowerS=0;
static int Blob::higherS=0;
static int Blob::lowerV=0;
static int Blob::higherV=0;

Blob::Blob(){
	RectData rectdata;

	//Initialise display windows; for debugging
	namedWindow("input");
	namedWindow("output");
	namedWindow("trackbar");
}

Blob::~Blob(){
	cv::destroyWindow("input");
	cv::destroyWindow("output");
}

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

/*Converts ROS image to CV image*/

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

/* Colour detection values */
colourDetection::colourDetection(){
	//Values are initialised as lowerH, lowerS, lowerV, higherH, higherS, higherV
	int yellow_values[6]{10, 0, 0, 79, 148, 255};
	int red_values[6]{0, 0, 100, 77, 195, 251};

	//For bounding box
	double max_area = 0;
	double areaThresh = 3000;

}

/* Detect colour values */
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
	outImg = reDraw(this->image);
	return outImg;

}

/* Perform colour detection */
cv::Mat colourDetection::reDraw(cv::Mat img){
	Mat out;
	cvtColor(img, out, CV_BGR2HSV);
	inRange(img, Scalar(lowerH,lowerS,lowerV), Scalar(higherH,higherS,higherV), out);
	Mat erodeEl = getStructuringElement(MORPH_RECT, cv::Size(9, 9));
	Mat dilateEl = getStructuringElement(MORPH_RECT, cv::Point(7, 7));
	erode(out, out, erodeEl, Point(-1, -1), 1);
	dilate(out, out, dilateEl, Point(-1, -1), 1);
	return out;
}

/* Find Bounding Box */
cv::Mat colourDetection::findBoundingBox(cv::Mat img){
	Mat out = image.clone();

	//Find x-center
	vector< cv::vector<Point> > contours;
	vector<cv::Vec4i> hierachy;

	findContours(out, contours, hierachy,
				 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Point2f center_max;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > areaThresh && area > max_area) {
			//Find the center using moments
			Moments mu;
			mu = moments(contours[i], false);
			double mu_area = mu.m00;
			center_max.x = mu.m10/mu_area;
			center_max.y = mu.m01/mu_area;
			max_area = area;

			//Find the bounding rect
			rectData.maxRect = minAreaRect(contours[i]);
		}
	}

	if (max_area > 0) {
		rectData.detected = true;
		rectData.center = center_max;

		Point2f points[4];
		rectData.maxRect.points(points);


	}
	return out;
}

/* Draw bounding box */
void colourDetection::drawBoundingBox(cv::Mat img){
	if (max_area > 0){
		circle(img, rectData.center, 20, Scalar(255, 255, 255));
		for (int i = 0; i < 4; i++) {
			Point2i pt1(int(points[i].x), int(points[i].y));
			Point2i pt2(int(points[(i+1)%4].x), int(points[(i+1)%4].y));
			line(img, pt1, pt2, Scalar(255, 255, 255));
		}
	}
	outImg = img;
}

/* Display images and windows */
void colourDetection::drawImage(){
	imshow("input", image);
	imshow("output", outImg);
}

/* Set up window settings */
void colourDetection::setWindowSettings(){
	createTrackbar("LowerH", "trackbar", &lowerH, 180, lowerHCallback, NULL);
    createTrackbar("UpperH", "trackbar", &higherH, 180, higherHCallback, NULL);

	createTrackbar("LowerS", "trackbar", &lowerS, 256, lowerSCallback, NULL);
    createTrackbar("UpperS", "trackbar", &higherS, 256, higherSCallback, NULL);

	createTrackbar("LowerV", "trackbar", &lowerV, 256, lowerVCallback, NULL);
    createTrackbar("UpperV", "trackbar", &higherV, 256, higherVCallback, NULL);
}

/** Callback methods **/
void lowerHCallback(int val, void *params) {
	Blob::lowerH = val;
	colourDetection::reDraw();
}
void higherHCallback(int val, void *params){
	this->higherH = val;
	reDraw();
}
void lowerSCallback(int val, void *params){
	this->lowerS = val;
	reDraw();
}
void higherSCallback(int val, void *params){
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


