/*
 * ControllerActionServer.cpp
 *
 *  Created on: 2013-05-27
 *      Author: gohew
 */
#include <ros/ros.h>
#include "ControllerActionServer.h"
#include <actionlib/server/simple_action_server.h>
#include <PID_Controller/ControllerAction.h>

ControllerActionServer::ControllerActionServer(std::string name) :
as_(nh_, name, boost::bind(&ControllerActionServer::executeCB, this, _1), false),
    action_name_(name)
{
	//register the goal and feeback callbacks
	//as_.registerGoalCallback(boost::bind(&goalCB));
	//as_.registerPreemptCallback(boost::bind(&preemptCB, this));

	as_.start();
}


void ControllerActionServer::premptCB()
{

}

void ControllerActionServer::executeCB(const PID_Controller::ControllerGoalConstPtr &goal)
{
	// helper variables
	ros::Rate r(1);
	bool success = true;

	// push_back the seeds for the fibonacci sequence
	feedback_.thruster = 1000;

	// publish info to the console for the user
	ROS_INFO(" Executing...");

	// check that preempt has not been requested by the client
	if (as_.isPreemptRequested() || !ros::ok())
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
		success = false;
	}
	feedback_.thruster = 1000;
	// publish the feedback
	as_.publishFeedback(feedback_);
	// this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
	r.sleep();
	if(success && _position > goal->setpoint)
	{
		result_.final_input = _position;
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		// set the action state to succeeded
		as_.setSucceeded(result_);
	} else as_.setAborted(result_);
}

void ControllerActionServer::updateState(float val)
{
	_position = val;
}

ControllerActionServer::~ControllerActionServer() {
	// TODO Auto-generated destructor stub
}

