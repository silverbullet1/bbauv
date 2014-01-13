//Header file for linefollowingstates

/* 
	linefollowerstates.h
	Header file for line following states
	Date created: 10 Jan 2014
	Author: Lynnette
*/

#ifndef LINEFOLLOWINGSTATES_H_
#define LINEFOLLOWINGSTATES_H_

#include <stdlib.h>
#include <string.h>

//Abstract base class for states
class State {
protected:
	cv::Mat inImage;
	std::string rectData;
public:
	State() {};
	State(cv::Mat image, std::string rectData) {
		this->rectData = rectData;
		this->inImage = image;
	}
	virtual ~State() {};
	virtual void setInputImage(cv::Mat);
	virtual void gotFrame(cv::Mat);
};

class LookForLine : public State{
private:
	std::string rectData;
public:
	LookForLine() {};
	LookForLine (cv::Mat image, std::string rectData) : State(image, rectData){};
	void gotFrame (cv::Mat);
};


#endif /* LINEFOLLOWINGSTATES_H_ */