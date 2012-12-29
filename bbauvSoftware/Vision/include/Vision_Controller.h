#ifndef VISION_CONTROLLER_H
#define VISION_CONTROLLER_H

#include "Data_Storage.h"
#include "Img_classifier.h"
#include "Img_Processor.h"
#include "Img_Storage.h"
#include "Img_Taker.h"
#include "Img_Viewer.h"
#include "bbauv_msgs/circles.h"

enum mode{MATCH_SHAPE,CIRCLE,RECTANGLE};


class Vision_controll{
private:
	ros::NodeHandle nh_;
	string reference;
	bbauv_msgs::circles cirr;
	Img_Taker *taker;
	Img_Storage *storage;
	Img_classfier *classy;
	Img_viewer *view;
	Data_Storage *data;
	Img_Processor *processor;
	ros::Publisher pub;
public:
	Vision_controll(int i,string name);
	~Vision_controll();
	void publish_Cir();
	void Init();
	void start();
	void Process_Mode(Mat &img,string refer,Mat &thres);

	int mode_controll;
	int color_controll;
private:
	Mat consider_Thres();

};
#endif
