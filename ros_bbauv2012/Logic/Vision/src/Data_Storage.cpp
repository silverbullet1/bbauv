#include "Data_Storage.h"

vector<Vec3f> Data_Storage::getCircleData(){
	return circles;
}

vector<vector<Point> > Data_Storage::get_contours(){
	return contours;
}

void Data_Storage::storeCircleData(vector<Vec3f> circl){
	circles=circl;
}

vector<vector<Point> > Data_Storage::getRect(){
	return Rectangles;
}

void Data_Storage::storeRec(vector<vector<Point> > rect){
	Rectangles=rect;
}
int Data_Storage::get_match_index(){
	return match_index;
}

void Data_Storage::store_match_index(int i){
	match_index=i;
}

void Data_Storage::storeContour(vector<vector<Point> > contrs){
	contours=contrs;
}
