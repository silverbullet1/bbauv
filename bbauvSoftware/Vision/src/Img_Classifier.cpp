#include "Img_classifier.h"

Img_classfier::Img_classfier(Img_Storage *store){
	storage=store;
}

Img_classfier::Img_classfier(Mat img,Img_Storage *store){
	storage=store;
	cur=img;
}

void Img_classfier::Process_Gray_and_Hsv(Mat cur){
	Mat grayTmp;
	Mat HsvTmp;
	cv::cvtColor(cur,grayTmp,CV_BGR2GRAY);
	cv::cvtColor(cur,HsvTmp,CV_BGR2HSV);

	storage->storeGreyScale(grayTmp);
	storage->storeHSV(HsvTmp);
}

void Img_classfier::thresGreen(Mat cur){
	Mat HsvTmp;

	Mat green_tmp;
	
	cv::cvtColor(cur,HsvTmp,CV_BGR2HSV);

	inRange(HsvTmp,Scalar(50,10,10,0),Scalar(87,255,255,0),green_tmp);
	
	storage->storeGreen(green_tmp);
}

void Img_classfier::thresOrange(Mat cur){
	Mat HsvTmp;

	Mat orange_tmp;
	cv::cvtColor(cur,HsvTmp,CV_BGR2HSV);

	inRange(HsvTmp,Scalar(0,10,20,0),Scalar(50,255,255,0),orange_tmp);

	storage->storeOrange(orange_tmp);
}

void Img_classfier::thresRed(Mat cur){
	Mat HsvTmp;

	Mat red_tmp1,red_tmp2;
	cv::cvtColor(cur,HsvTmp,CV_BGR2HSV);

	inRange(HsvTmp,Scalar(160,10,20,0),Scalar(180,255,255,0),red_tmp1);
	inRange(HsvTmp,Scalar(0,10,20,0),Scalar(5,255,255,0),red_tmp2);

	red_tmp1=red_tmp1 | red_tmp2;

	storage->storeRed(red_tmp1);
}

void Img_classfier::Classify_Img(Mat cur){
	storage->storeCurrent(cur);
	GaussianBlur(cur,cur,Size(9,9),2,2);
	Process_Gray_and_Hsv(cur);
	thresGreen(cur);
	thresOrange(cur);
	thresRed(cur);
}
