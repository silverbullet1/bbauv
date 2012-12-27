#ifndef IMG_PROCESSOR_H
#define IMG_PROCESSOR_H

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
#include "Img_Storage.h"
#include "Data_Storage.h"

 
using namespace std;
using namespace cv;

class Img_Processor{
private:
	Data_Storage *data_store;
	Img_Storage *storage;

public:
	Img_Processor(Img_Storage *store,Data_Storage *data_st);
	Mat pre_process(Mat img);
	void find_circles(Mat img);

	vector<vector<Point> > findSquaresInImage(Mat _image,bool binary);
	void drawSquares( Mat img, vector<vector<Point> > squares);
	void MatchShape(Mat img,string reference);
private:
	void findCont(Mat &img1,vector<vector<Point> > &contours,vector<Vec4i> &hierarchy,Mat &output);
	double angle( Point pt1, Point pt2, Point pt0 );
	int findBiggestAread(vector<vector<Point> >contours);
	int CompareCont(vector<vector<Point> >contours1,vector<vector<Point> >contoursRef,int biggest);
	
};

#endif
