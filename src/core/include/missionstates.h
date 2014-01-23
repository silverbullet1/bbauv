/*
 * missionstates.h
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & thien
 */

#ifndef MISSIONSTATES_H_
#define MISSIONSTATES_H_

#include <ros/ros.h>
#include <bbauv_msgs/controller.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
extern ros::Publisher statePub;		//Wanna publish the states? Like enable or disable...

//Abstract base class for states
class State : public boost::enable_shared_from_this<State>{
public:
	State(){};
	virtual ~State() {};
private:
	bool enabled = false;	//Whether the state is enabled or not
};

//Start state
class StartState : public boost::enable_shared_from_this<State>{
public:
	StartState(){};
};

//Start line follower
class LineFollowingState : public boost::enable_shared_from_this<State>{
public:
	LineFollowingState(boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

//Searching for bucket blob
class BucketSearchState : public boost::enable_shared_from_this<State>{
public:
	BucketSearchState(boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

//Enable the bucket node
class BucketStartState : public boost::enable_shared_from_this<State>{
public:
	BucketStartState(boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

//Enable the flare state
class FlareState : public boost::enable_shared_from_this<State>{
public:
	FlareState(boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

//Pass to acoustics
class AcousticState : public boost::enable_shared_from_this<State>{
public:
	AcousticState(boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

//Finish run, surface
class SurfaceState : public boost::enable_shared_from_this<State>{
public:
	SurfaceState(double heading, boost::shared_ptr<State> nextState);
private:
	boost::shared_ptr<State> nextState;
};

#endif /* MISSIONSTATES_H_ */
