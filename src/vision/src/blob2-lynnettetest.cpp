//Testing blob2.cpp

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int lowerH=0, lowerS=0, lowerV=0;
int upperH=95, upperS=148, upperV=255;

cv::Mat image;

using namespace cv;


void drawImage(){
	cv::Mat red_image;
	cv::cvtColor(image, red_image, CV_BGR2HSV);
	cv::inRange(image, cv::Scalar(lowerH,lowerS,lowerV), cv::Scalar(upperH,upperS,upperV), red_image);
	cv::Mat erodeEl = getStructuringElement(MORPH_RECT, cv::Size(9, 9));
	cv::Mat dilateEl = getStructuringElement(MORPH_RECT, cv::Point(7, 7));
	cv::erode(red_image, red_image, erodeEl, Point(-1, -1), 1);
	cv::dilate(red_image, red_image, dilateEl, Point(-1, -1), 1);
	cv::imshow("blob-out", red_image);
}

void lowerHCallback(int val, void *params){
	lowerH = val;
	drawImage();
}

void upperHCallback(int val, void *params){
	upperH = val;
	drawImage();
}

void lowerSCallback(int val, void *params){
	lowerS = val;
	drawImage();
}

void upperSCallback(int val, void *params){
	upperS = val;
	drawImage();
}

void lowerVCallback(int val, void *params){
	lowerV = val;
	drawImage();
}

void upperVCallback(int val, void *params){
	upperV = val;
	drawImage();
}

void setWindowSettings(){
	cv::namedWindow("blob-in");

	cv::namedWindow("blob-out");

	cv::createTrackbar("LowerH", "blob-in", &lowerH, 180, lowerHCallback, NULL);
    cv::createTrackbar("UpperH", "blob-in", &upperH, 180, upperHCallback, NULL);

	cv::createTrackbar("LowerS", "blob-in", &lowerS, 256, lowerSCallback, NULL);
    cv::createTrackbar("UpperS", "blob-in", &upperS, 256, upperSCallback, NULL);

	cv::createTrackbar("LowerV", "blob-in", &lowerV, 256, lowerVCallback, NULL);
    cv::createTrackbar("UpperV", "blob-in", &upperV, 256, upperVCallback, NULL); 
}

int main(int argc, char** argv){
	image = cv::imread("pic2.jpg");
	cv::resize(image, image, cv::Size(600,480));
	
	setWindowSettings();
	drawImage();
	cv::imshow("blob-in", image);


	cv::waitKey();
	return (0);
}
