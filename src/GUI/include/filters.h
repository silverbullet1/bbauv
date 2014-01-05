#ifndef FILTERS_H_
#define FILTERS_H_

#define NUM_FILTERS 1

typedef cv::Mat (* filter_t) (cv::Mat image);

cv::Mat Canny_filter(cv::Mat image);

extern filter_t front_filters[NUM_FILTERS];
extern filter_t bottom_filters[NUM_FILTERS];

#endif /* FILTERS_H_ */
