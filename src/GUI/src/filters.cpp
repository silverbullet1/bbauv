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

//Container for Filters
const int FiltersContainer::NUM_FILTERS = 2;

FiltersContainer::FiltersContainer() {
	//Instantiate filters
	filter1 = new CannyFilter();
	filter2 = new AdaptiveThresholdFilter();

	Filter* front_filters_array[] = { filter1, filter2 };
	Filter* bottom_filters_array[] = { filter1, filter2 };

	front_filters.insert(front_filters.end(),
						 front_filters_array, front_filters_array + NUM_FILTERS);
	bottom_filters.insert(bottom_filters.end(),
						  bottom_filters_array, bottom_filters_array + NUM_FILTERS);
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
}
