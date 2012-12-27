#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;
int main(){
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
	// the recorded video will be closed automatically in the VideoWriter destructor
	return 0;
}