#include "Img_Taker.h"

Img_Taker::Img_Taker(string name){
	id=-1;
	Video_name=name;
	cap.open(name);

	frameskip = 1;
	currentFrame = 0;
}
Img_Taker::Img_Taker(int i){
	id=i;
	cap.open(i);

	frameskip = 1;
	currentFrame = 0;
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
		else {
			cap.open(Video_name);
			currentFrame = 0;
		}

	if (id == -1) {
		// Only skip frames when reading from a video
		cap.set(CV_CAP_PROP_POS_FRAMES, currentFrame);
		currentFrame += frameskip;
	}

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

void Img_Taker::setFrameskip(int skip) {
	frameskip = std::max(0, skip);
}

int Img_Taker::getFrameskip() {
	return frameskip;
}
