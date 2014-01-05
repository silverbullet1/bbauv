#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "filters.h"

cv::Mat Canny_filter(cv::Mat image) {
	cv::Mat out;

	cv::GaussianBlur(image, out, cv::Size(5, 5), 0, 0);
	cv::Canny(out, out, 100, 250, 3);

	return out;
}

cv::Mat adaptiveThresh_filter(cv::Mat image) {
	cv::Mat out;

	cv::cvtColor(image, out, CV_BGR2GRAY);
	cv::adaptiveThreshold(out, out, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 71, 10);

	return out;
}

filter_t front_filters[NUM_FILTERS] = { Canny_filter } ;
filter_t bottom_filters[NUM_FILTERS] = { adaptiveThresh_filter };
