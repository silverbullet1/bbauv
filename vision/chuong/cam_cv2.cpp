#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;
int threshL=100;

int threshH=300;

void computeCountour(Mat img){
	
	namedWindow("Canny control", CV_WINDOW_AUTOSIZE);

	//resizeWindow("Canny control", 200, 200);
	createTrackbar("thresh canny low","Canny control",&threshL,500,0);
	createTrackbar("thresh canny high","Canny control",&threshH,500,0);
	Mat canny;
	Canny(img,canny,threshL,threshH,3);
	namedWindow("Canny_");
	imshow("Canny_",canny);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(canny,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{ approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
	boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	minEnclosingCircle( contours_poly[i], center[i], radius[i] );
	}
	/// Draw contours
	Mat drawing = Mat::zeros( canny.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		double center_x;
		double center_y;
		double area;
		center_x=(boundRect[i].tl().x +boundRect[i].br().x)/2.0;
		center_y=(boundRect[i].tl().y +boundRect[i].br().y)/2.0;
		area=boundRect[i].height * boundRect[i].width;
		//if (area>1000.0){
			//printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
			Scalar color = Scalar( 0, 0, 255 );
			drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
			//circle( drawing, mc[i], 4, Scalar(0,255,0), -1, 8, 0 );
			circle(drawing,Point(center_x,center_y),4,Scalar(0,255,0), -1, 8, 0 );
			//rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );

			circle( drawing, center[i], (int)radius[i], color, 1, 8, 0 );
		//}

	}
	namedWindow("Contour");
	imshow("Contour",drawing);
}
int main(int argc, char* argv[]){
	//VideoCapture cap("E:/frames/SONIA_RUN.flv"); // open the default camera
	//VideoCapture cap(1);
	VideoCapture cap(0);
	if(!cap.isOpened())  // check if we succeeded
		return -1;

	Mat edges;
	//namedWindow("edges",1);
	int frameskip = 1;
	namedWindow("HSV Controls", CV_WINDOW_AUTOSIZE);

	resizeWindow("HSV Controls", 500, 500);

	/// Create Trackbar to choose type of Threshold

	int hs=0, he=0, ss=10, se=255, vs=10, ve=255;

	createTrackbar( "hStart: ", "HSV Controls", &hs, 180, 0);
	createTrackbar( "hEnd:   ", "HSV Controls", &he, 180, 0);
	createTrackbar( "sStart: ", "HSV Controls", &ss, 255, 0);
	createTrackbar( "sEnd:   ", "HSV Controls", &se, 255, 0);
	createTrackbar( "vStart: ", "HSV Controls", &vs, 255, 0);
	createTrackbar( "vEnd:   ", "HSV Controls", &ve, 255, 0);
	for(;;)
	{
		Mat frame;
		int frames = frameskip;
		while (frames-- > 0) {
			cap >> frame;
		}
		if (frame.cols==0)
			break;
		//cvtColor(frame, edges, CV_BGR2GRAY);
		//pyrDown(frame, frame, Size(frame.cols/2, frame.rows/2));
		Mat mask;
		GaussianBlur(frame,frame,Size(9,9),2,2);
		cvtColor(frame,mask,CV_BGR2HSV);
		namedWindow("hsv",1);
		imshow("hsv",mask);
		//IplImage *img2=cvCloneImage(&(IplImage)buoy);
		//GaussianBlur(mask,mask,cv::Size(9,9),1.5);
		
		Mat mask2;
		inRange(mask,Scalar(hs,ss,vs,0),Scalar(he,se,ve,0),mask2);
		//inRange(mask,Scalar(35,10,10,0),Scalar(80,255,255,0),mask2);
		erode(mask2,mask2,Mat());
		dilate(mask2,mask2,Mat());
		GaussianBlur( mask2, mask2, Size(9, 9), 2, 2 );
		computeCountour(mask2);
		namedWindow("mask",1);
		imshow("mask",mask2);
		Mat gray;

		vector<Vec3f> circles;
		HoughCircles(mask2,circles,CV_HOUGH_GRADIENT,1,mask2.rows/8,300,20,5,400);
		for( size_t i = 0; i < circles.size(); i++ )
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
		}
		namedWindow("buoyCir",1);
		imshow("buoyCir",frame);
		int c;
		c = waitKey( 20 );
		if( (char)c == 27 )
		{ break; }
		switch (c) {
		case '=':
			frameskip += 10;
			break;
		case '-':
			frameskip -= 10;
			frameskip = std::max(0, frameskip);
			break;
		case '0':
			frameskip = 1;
			break;
		case ' ':
			frameskip = !frameskip;
			break;
		}
	}


	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}