/* 
	filters.cpp
	Filters for vision task
	Date created: 8 Jan 2014
	Author: Thein
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "filters.h"

using namespace cv;

//Abstract Base Filter
void Filter::setInputImage(Mat image) {
	this->inImage = image;
}

//Canny Filter
std::string CannyFilter::name = "Canny";

Mat CannyFilter::getOutputImage() {
	Mat out;

	GaussianBlur(this->inImage, out, cv::Size(5, 5), 0, 0);
	Canny(out, out, 100, 250, 3);

	return out;
}


//Adaptive Threshold Filter
std::string AdaptiveThresholdFilter::name = "Adaptive Threshold";

Mat AdaptiveThresholdFilter::getOutputImage() {
	cv::Mat out;

	cvtColor(this->inImage, out, CV_BGR2GRAY);
	adaptiveThreshold(out, out, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 71, 10);

	return out;
}

//Detecting center of blackline
std::string BlackLineCenter::name = "Black Line Center";

BlackLineCenter::BlackLineCenter() {
	thVal = 30;
	areaThresh = 2000;
}

Mat BlackLineCenter::getOutputImage() {
	Mat greyImg;
	cvtColor(inImage, greyImg, CV_BGR2GRAY);
	resize(greyImg, greyImg, Size(640, 480));
	//ROI
	Mat roiImg;
	Rect roi(0, 190, 640, 100);
	greyImg(roi).copyTo(roiImg);

	//Thresholding and noise removal
	GaussianBlur(roiImg, roiImg, Size(5, 5), 0, 0);
	threshold(roiImg, roiImg, thVal, 255, THRESH_BINARY_INV);
	Mat erodeEl = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat dilateEl = getStructuringElement(MORPH_RECT, Point(5, 5));
	erode(roiImg, roiImg, erodeEl);
	dilate(roiImg, roiImg, dilateEl);

	//Find x-center
	cv::Mat out = inImage.clone();
	resize(out, out, Size(640, 480));
	cv::vector< cv::vector<Point> > contours;
	cv::vector<cv::Vec4i> hierachy;

	findContours(roiImg, contours, hierachy,
				 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (size_t i = 0; i < contours.size(); i++) {
		float area = contourArea(contours[i]);
		if (area > areaThresh) {
			Moments mu;
			mu = moments(contours[i], false);
			Point2f center(mu.m10/mu.m00, 240);
			circle(out, center, 5, Scalar(0, 255, 0));
		}
	}

	return out;
}

//Container for Filters
const int FiltersContainer::NUM_FRONT_FILTERS = 2;
const int FiltersContainer::NUM_BOTTOM_FILTERS = 3;

FiltersContainer::FiltersContainer() {
	//Instantiate filters
	filter1 = new CannyFilter();
	filter2 = new AdaptiveThresholdFilter();
	filter3 = new BlackLineCenter();

	Filter* front_filters_array[] = { filter1, filter2 };
	Filter* bottom_filters_array[] = { filter1, filter2, filter3 };

	front_filters.insert(front_filters.end(),
						 front_filters_array, front_filters_array + NUM_FRONT_FILTERS);
	bottom_filters.insert(bottom_filters.end(),
						  bottom_filters_array, bottom_filters_array + NUM_BOTTOM_FILTERS);
}

std::vector<Filter*> FiltersContainer::getFrontFilters() {
	return this->front_filters;
}

std::vector<Filter*> FiltersContainer::getBottomFilters() {
	return this->bottom_filters;
}

FiltersContainer::~FiltersContainer() {
	delete filter1;
	delete filter2;
	delete filter3;
}
