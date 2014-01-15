//Header file for linefollowingstates

/* 
	linefollowerstates.h
	Header file for line following states
	Date created: 10 Jan 2014
	Author: Lynnette & Thien
*/

#ifndef LINEFOLLOWINGSTATES_H_
#define LINEFOLLOWINGSTATES_H_

#include <stdlib.h>
#include <string.h>

//Structure for storing data about blackline
struct RectData {
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Abstract base class for states
class State {
public:
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData);
};

class LookForLineState : public State{
public:
	boost::shared_ptr<State> gotFrame (cv::Mat, RectData rectData);
};

class DiveState : public State {
private:
	double transitionTime;
	boost::shared_ptr<State> nextState;
public:
	DiveState (double secondsToDive, boost::shared_ptr<State> nextState);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

class SurfaceState : public State {
public:
	SurfaceState(double heading);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

class StraightLineState : public State {
public:
	StraightLineState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};


#endif /* LINEFOLLOWINGSTATES_H_ */
