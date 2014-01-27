//Header file for linefollowingstates

/* 
	linefollowerstates.h
	Header file for line following states
	Date created: 10 Jan 2014
	Author: Lynnette & Thien
*/

#ifndef LINEFOLLOWINGSTATES_H_
#define LINEFOLLOWINGSTATES_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <bbauv_msgs/controller.h>
#include <bbauv_msgs/ControllerAction.h>
#include <bbauv_msgs/ControllerActionGoal.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
//extern const int endTime;
extern const int DEPTH_POINT;
extern const double secondsToRun;
extern const double x_strip_threshold;
extern actionlib::SimpleActionClient <bbauv_msgs::ControllerAction> ac;

inline void publishMovement(const bbauv_msgs::ControllerGoal goal) {
	ac.sendGoal(goal);
}

//inline void publishMovement(const bbauv_msgs::controller goal) {
//	movementPub.publish(goal);
//}

//Function to normalise heading in degrees
inline double normHeading(double heading)
{
	if (heading > 360.0) { return heading - 360.0; }
	else if (heading < 0.0) { return heading + 360.0; }
	else { return heading; }
}

//Structure for storing data about blackline
struct RectData {
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Abstract base class for states
class State : public boost::enable_shared_from_this<State> {
public:
	State() {};
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

class LookForLineState : public State {
public:
	LookForLineState();
	boost::shared_ptr<State> gotFrame (cv::Mat, RectData rectData);
};

class TemporaryState : public State {
private:
	double transitionTime;
	boost::shared_ptr<State> nextState;
	double speed;
public:
	TemporaryState(double secondsToReverse,
				   boost::shared_ptr<State> nextState,
				   double speed);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData);
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
