#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H
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


using namespace std;
using namespace cv;

class Data_Storage{
private:
	vector<Vec3f> circles;
	vector<vector<Point> > Rectangles;
	vector<vector<Point> > contours;

	int match_index;
public:
	Data_Storage(){};
	vector<Vec3f> getCircleData();
	vector<vector<Point> > getRect();
	vector<vector<Point> > get_contours();
	int get_match_index();

	void store_match_index(int i);
	void storeCircleData(vector<Vec3f> circ);
	void storeRec(vector<vector<Point> > rect);
	void storeContour(vector<vector<Point> > contrs);
};
#endif
