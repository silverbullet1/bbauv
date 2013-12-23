#ifndef IMG_STORAGE_H
#define IMG_STORAGE_H

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



class Img_Storage{
private:
	Mat current;
	Mat grayScale;
	Mat red_thres;
	Mat orange_thres;
	Mat green_thres;
	Mat hsv_cur;
	Mat cir_current;
public:
	Img_Storage(){};
	Mat getCurrent();
	Mat getGrayScale();
	Mat getRedThresholing();
	Mat getOrangeThresHolding();
	Mat getGreenThresHolding();
	Mat getHSV_Cur();
	Mat getCircle_Frame();

	void storeHSV(Mat _hsv_);
	void storeCurrent(Mat cur);
	void storeGreyScale(Mat grey);
	void storeRed(Mat red);
	void storeGreen(Mat green);
	void storeOrange(Mat orange);
	void storeCircleFrame(Mat circle);
};
#endif
