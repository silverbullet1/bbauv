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

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
class Recorder{
private:
	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	int recor;
	VideoWriter record;
	bool isOpen;
public:
	Recorder(): it_(nh_) {
		image_sub_= it_.subscribe("/camera/rgb/image_color"
, 1, &Recorder::imageCb, this);
		namedWindow("Record", CV_WINDOW_AUTOSIZE);
		recor=0;
		isOpen=false;

	}
	~Recorder(){
		destroyWindow("Record");
	}
	void imageCb(const sensor_msgs::ImageConstPtr &msg){
		createTrackbar("record", "Record", &recor,1,0);
		cv_bridge::CvImagePtr cv_ptr;
   		 try
   		{
     			 cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    		}
    		catch (cv_bridge::Exception& e)
   		 {
    		  ROS_ERROR("cv_bridge exception: %s", e.what());
     		 return;
    		}
		Mat frame=cv_ptr->image;
		if (!isOpen){
			record.open("Write_test.avi",CV_FOURCC('M','J','P','G'),30, frame.size(),true);
			isOpen=true;
		}
		if( !record.isOpened() ) {
			printf("VideoWriter failed to open!\n");
			
		}	
		imshow("Record",frame);
		// add frame to recorded video
		if (recor==1){
			record << frame; 
			ROS_INFO("Writing video");
		}
		cv::waitKey(3);

	}

};
int main(int argc, char** argv){
	/*ros::init(argc, argv, "Write_test");
	VideoCapture capture(1);
	if (!capture.isOpened())
		return -1;

	Mat frame;
	capture >> frame; // get first frame for size
	namedWindow("Record", CV_WINDOW_AUTOSIZE);

	//resizeWindow("Record", 500, 500);
	int recor=0;
	createTrackbar( "record   ", "Record", &recor, 1, 0);
	// record video
	VideoWriter record("Write_test.avi", CV_FOURCC('D','I','V','X'), 30, frame.size(), true);
	if( !record.isOpened() ) {
		printf("VideoWriter failed to open!\n");
		return -1;
	}

	namedWindow("video",1);
	for(;;)
	{
		// get a new frame from camera
		capture >> frame; 

		// show frame on screen
		imshow("video", frame); 

		// add frame to recorded video
		if (recor==1)
			record << frame; 

		if(waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	// the recorded video will be closed automatically in the VideoWriter destructor*/
	ros::init(argc, argv, "image_recorder");
  	Recorder ic;
  	ros::spin();
	return 0;
}
