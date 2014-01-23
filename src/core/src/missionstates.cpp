/*
 * missionstates.cpp
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & Thien
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string.h>

#include "missionstates.h"

StartState::StartState(){
	ROS_INFO("Starting to go!");
}

LineFollowingState::LineFollowingState(boost::shared_ptr<State> nextState){
	ROS_INFO("Line Following State Enabled");
	this->nextState = nextState;
}

BucketSearchState::BucketSearchState(boost::shared_ptr<State> nextState){
	ROS_INFO("Bucket Searching State Enabled");
	this->nextState = nextState;
}

BucketStartState::BucketStartState(boost::shared_ptr<State> nextState){
	ROS_INFO("Bucket Starting State Enabled");
	this->nextState = nextState;
}

FlareState::FlareState(boost::shared_ptr<State> nextState){
	ROS_INFO("Flare State Enabled");
	this->nextState = nextState;
}

AcousticState::AcousticState(boost::shared_ptr<State> nextState){
	ROS_INFO("Acoustics State Enabled");
	this->nextState = nextState;
}

SurfaceState::SurfaceState(){
	ROS_INFO("End of run! Good job!");
}


