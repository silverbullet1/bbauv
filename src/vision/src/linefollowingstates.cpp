//States for the linefollower

/* 
	linefollowerstates.cpp
	States for line follower
	Date created: 10 Jan 2014
	Author: Lynnette & Thein
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string.h>
#include <queue>

#include "linefollowingstates.h"

std::queue<double> hist; //queue to store previous angles;

// Look for line state
LookForLineState::LookForLineState() {
	ROS_INFO("Looking for line");
}

boost::shared_ptr<State> LookForLineState::gotFrame(cv::Mat image, RectData rectData) {
	if (rectData.detected)
		return boost::shared_ptr<State>(new StraightLineState());

	bbauv_msgs::controller msg;
	msg.depth_setpoint = DEPTH_POINT;
	//msg.heading_setpoint = rectData.heading + 10;
	publishMovement(msg);
	return shared_from_this();
}

//Temporary State
TemporaryState::TemporaryState(double secondsToReverse,
							   boost::shared_ptr<State> nextState,
							   double speed = -0.6) {
	ROS_INFO("Reversing for %lf secs", secondsToReverse);
	transitionTime = ros::Time::now().toSec() + secondsToReverse;
	this->nextState = nextState;
	this->speed = speed;
}

boost::shared_ptr<State> TemporaryState::gotFrame(cv::Mat image, RectData rectData) {
	if (ros::Time::now().toSec() > transitionTime)
		return nextState;

	bbauv_msgs::controller msg;
	msg.depth_setpoint = DEPTH_POINT;
	msg.heading_setpoint = rectData.heading;
	msg.forward_setpoint = speed;
	publishMovement(msg);
	return shared_from_this();
}

//Dive State
DiveState::DiveState (double secondsToDive, boost::shared_ptr<State> nextState) {
	ROS_INFO("Diving for %lf secs", secondsToDive);
	transitionTime = ros::Time::now().toSec() + secondsToDive;
	this->nextState = nextState;
}

boost::shared_ptr<State> DiveState::gotFrame(cv::Mat image, RectData rectData) {
	if (ros::Time::now().toSec() > transitionTime)
		return nextState;

	//Dive
	bbauv_msgs::controller msg;
	msg.depth_setpoint = DEPTH_POINT;
	msg.heading_setpoint = rectData.heading;
	publishMovement(msg);
	return shared_from_this();
}

//Surface State
SurfaceState::SurfaceState (double heading) {
	ROS_INFO("Surfacing");

    bbauv_msgs::controller msg;
    msg.depth_setpoint = 0.2;
    msg.heading_setpoint = heading;
    publishMovement(msg);
}

boost::shared_ptr<State> SurfaceState::gotFrame(cv::Mat, RectData) {
	return shared_from_this();
}

//Straight Line State
StraightLineState::StraightLineState() {
	ROS_INFO("Following a straight line");
}

boost::shared_ptr<State> StraightLineState::gotFrame(cv::Mat image, RectData rectData) {
	if (!rectData.detected) {
		boost::shared_ptr<State> lNextState(new LookForLineState());
		return boost::shared_ptr<State>(new TemporaryState(0.5, lNextState));
	}

	int screen_width = image.cols;
	int screen_center_x = screen_width / 2;

	double delta_x = (double) (rectData.center.x - screen_center_x) / screen_width;

	bbauv_msgs::controller msg;
	msg.depth_setpoint = DEPTH_POINT;

    //if the rect is too far off centre, do aggressive sidemove
	if (abs(delta_x) > 0.3) {
		ROS_INFO("Box too far off centre! Aggressive sidemove");
		msg.heading_setpoint = normHeading(rectData.heading - rectData.angle);
		msg.sidemove_setpoint = delta_x < 0 ? 1.0 : -1.0;
		publishMovement(msg);
		return shared_from_this();
	}

	std::cout << rectData.angle << std::endl;

	//Based on previous angle, determine if the new angle should be pointing the opposite direction
	if (!hist.empty()) {
		double oppAngle = rectData.angle > 0 ? rectData.angle - 180 : rectData.angle + 180;
		if (abs(rectData.angle - hist.back()) > abs(oppAngle - hist.back())) {
			rectData.angle = oppAngle;
		}
	}

	hist.push(rectData.angle);

	if (delta_x < -x_strip_threshold) {
		msg.sidemove_setpoint = 0.5;
	} else if (delta_x > x_strip_threshold) {
		msg.sidemove_setpoint = -0.5;
	}

	if (abs(rectData.angle) < 10) {
		//Keep moving forward
		msg.heading_setpoint = rectData.heading;
		msg.forward_setpoint = 0.9;
		ROS_INFO("Forward!");
	} else {
		if (msg.sidemove_setpoint == 0 && abs(rectData.angle > 10)) {
			msg.sidemove_setpoint = rectData.angle / 60 * 0.2;
		}

		double angle_diff = rectData.angle;
		if (angle_diff > 30) {
			angle_diff = rectData.angle > 0 ? 30.0 : -30.0;
		}
		msg.heading_setpoint = normHeading(rectData.heading - angle_diff);
	}
	publishMovement(msg);
	return shared_from_this();
}
