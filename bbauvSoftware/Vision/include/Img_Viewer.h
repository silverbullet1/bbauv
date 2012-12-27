#ifndef IMG_VIEWER_H
#define IMG_VIEWER_H
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
#include "Img_Storage.h"
using namespace std;
using namespace cv;
enum colors{RED,GREEN,ORANGE};
class Img_viewer{
private:
	Img_Storage *storage;

	string thres_color;
	string prev_thres;
public:
	Img_viewer(Img_Storage *store);
	void show_Current();
	void show_HSV();
	void show_Gray();
	void show_Green();
	void show_Red();
	void show_Orange();
	void show_Cirle_Frame();
	void show_Thres(int thres_index);
};
#endif
