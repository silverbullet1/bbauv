#ifndef IMG_TAKER_H
#define IMG_TAKER_H

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include<math.h>
#include <cxcore.h>
#include <highgui.h>

#include <vector>

using namespace std;
using namespace cv;

class Img_Taker{
private:
	VideoCapture cap;
	Mat frame;
	string Video_name;
	int id;
public:
	Img_Taker(){};
	Img_Taker(string name);
	Img_Taker(int i);
	void SetCapture(string name);
	void SetCapture(int i);
	VideoCapture GetCapture();
	Mat GetsequenceFrame(int i,string name);//If i=-1 then take name
	Mat getFrame();
};
#endif
