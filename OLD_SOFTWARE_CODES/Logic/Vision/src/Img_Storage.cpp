#include "Img_Storage.h"

Mat Img_Storage::getCurrent(){
	return current;
}

Mat Img_Storage::getGrayScale(){
	return grayScale;
}

Mat Img_Storage::getGreenThresHolding(){
	return green_thres;
}
Mat Img_Storage::getOrangeThresHolding(){
	return orange_thres;
}

Mat Img_Storage::getRedThresholing(){
	return red_thres;
}

Mat Img_Storage::getHSV_Cur(){
	return hsv_cur;
}

Mat Img_Storage::getCircle_Frame(){
	return cir_current;
}

void Img_Storage::storeCircleFrame(Mat circle){
	cir_current=circle;
}
void Img_Storage::storeHSV(Mat _hsv_){
	hsv_cur=_hsv_;
}
void Img_Storage::storeCurrent(Mat cur){
	cur.copyTo(current);
}
void Img_Storage::storeGreen(Mat green){
	green.copyTo(green_thres);
}

void Img_Storage::storeGreyScale(Mat grey){
	grey.copyTo(grey);
}

void Img_Storage::storeRed(Mat red){
	red.copyTo(red_thres);
}
void Img_Storage::storeOrange(Mat orange){
	orange.copyTo(orange_thres);
}