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
int Blob::getHigherH() {  } ;
void Blob::setLowerS(int lowerS);
int Blob::getLowerS();
void Blob::setHigherS(int higherS);
int Blob::getHigherS();
void Blob::setLowerV(int lowerV);
int Blob::getLowerV();
void Blob::setHigherV(int higherV);
int Blob:getHigherV();

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



