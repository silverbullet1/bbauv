#include "Img_Subscriber.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

void Img_Subscriber::image_callback(const sensor_msgs::ImageConstPtr& message) {
	frame = cv_bridge::toCvCopy(message)->image;
}

Img_Subscriber::Img_Subscriber(string topic) {
	topic_name = topic;

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_sub = it.subscribe(topic, 1, &Img_Subscriber::image_callback, this);

	frame = Mat::zeros(240, 320, CV_8UC3);
}

Mat Img_Subscriber::getFrame() {
	return frame;
}

