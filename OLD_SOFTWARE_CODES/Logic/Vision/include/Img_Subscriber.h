#ifndef IMG_SUBSCRIBER_H
#define IMG_SUBSCRIBER_H

#include <string>
#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "Img_Taker_Base.h"

class Img_Subscriber : public Img_Taker_Base {
private:
	cv::Mat frame;
	std::string topic_name;
	image_transport::Subscriber image_sub;

	void image_callback(const sensor_msgs::ImageConstPtr& message);

public:
	Img_Subscriber(std::string topic);
	cv::Mat getFrame();

	// Dummy functions
	void setFrameskip(int skip) { }
	int getFrameskip() { return 0; }
};
#endif//IMG_SUBSCRIBER_H
