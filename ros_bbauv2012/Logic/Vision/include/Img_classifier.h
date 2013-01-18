#ifndef IMG_CLASSIFIER
#define IMG_CLASSIFIER

#include "Img_Storage.h"
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
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
of robots,here the device used is a ardrone(quad-rotor).*/
 
using namespace std;
using namespace cv;


class Img_classfier{
private:
	Mat cur;
	Img_Storage *storage;
public:
	Img_classfier(Img_Storage *store);
	Img_classfier(Mat img,Img_Storage *store);

	void Classify_Img(Mat cur);
	void Process_Gray_and_Hsv(Mat cur);
	void thresGreen(Mat cur);
	void thresOrange(Mat cur);
	void thresRed(Mat cur);
};

#endif
