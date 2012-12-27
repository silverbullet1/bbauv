#include "Img_Viewer.h"

Img_viewer::Img_viewer(Img_Storage *store){
	storage=store;
}

void Img_viewer::show_Current(){
	Mat cur=storage->getCurrent();
	namedWindow("Current frame");
	imshow("Current frame",cur);
}

void Img_viewer::show_Gray(){
	Mat gray_tmp=storage->getGrayScale();
	namedWindow("Gray frame");
	imshow("Gray frame",gray_tmp);
}

void Img_viewer::show_HSV(){
	Mat hsv_tmp=storage->getHSV_Cur();
	namedWindow("HSV frame");
	imshow("HSV frame",hsv_tmp);
}

void Img_viewer::show_Green(){
	Mat green_tmp=storage->getGreenThresHolding();
	namedWindow(thres_color);
	imshow(thres_color,green_tmp);
}

void Img_viewer::show_Red(){
	Mat red_tmp=storage->getRedThresholing();
	namedWindow(thres_color);
	imshow(thres_color,red_tmp);
}

void Img_viewer::show_Cirle_Frame(){
	Mat cir_tmp=storage->getCircle_Frame();
	namedWindow("Circle_frame");
	imshow("Circle_frame",cir_tmp);
}
void Img_viewer::show_Orange(){
	Mat orange_tmp=storage->getOrangeThresHolding();
	namedWindow(thres_color);
	imshow(thres_color,orange_tmp);
}

void Img_viewer::show_Thres(int thres_index){
	
	switch(thres_index){
	case RED:
		thres_color="Red_frame";
		show_Red();
		break;
	case GREEN:
		thres_color="Green_frame";
		show_Green();
		break;
	case ORANGE:
		thres_color="Orange_frame";
		show_Orange();
		break;
	}
	if (thres_color!=prev_thres){
		destroyWindow(prev_thres);
		prev_thres=thres_color;
	}
}