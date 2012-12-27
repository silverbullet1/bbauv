#include "Img_Taker.h"

Img_Taker::Img_Taker(string name){
	id=-1;
	Video_name=name;
	cap.open(name);
}
Img_Taker::Img_Taker(int i){
	id=i;
	cap.open(i);
}

void Img_Taker::SetCapture(string name){
	Video_name=name;
	cap.open(name);
}
void Img_Taker::SetCapture(int i){
	id=i;
	cap.open(i);
}

VideoCapture Img_Taker::GetCapture(){
	return cap;
}

Mat Img_Taker::GetsequenceFrame(int i,string name){
	if (!cap.isOpened())
		if (i!=-1)
			cap.open(i);
		else
			cap.open(name);
	bool nxtFrame;
	nxtFrame=cap.read(frame);
	if (nxtFrame==false){
		cap.release();
			if (i!=-1)
				cap.open(i);
			else
				cap.open(name);
		cap>>frame;
	}
	return frame;
}

Mat Img_Taker::getFrame(){
	if (!cap.isOpened())
		if (id!=-1)
			cap.open(id);
		else
			cap.open(Video_name);
	bool nxtFrame;
	nxtFrame=cap.read(frame);
	if (nxtFrame==false){
		cap.release();
		if (id!=-1)
			cap.open(id);
		else
			cap.open(Video_name);
		cap>>frame;
	}
	return frame;
}