/*
 * ControllerActionServer.cpp
 *
 *  Created on: 2013-05-27
 *      Author: gohew
 */
#include <ros/ros.h>
#include "ControllerActionServer.h"
#include <actionlib/server/simple_action_server.h>
#include <bbauv_msgs/ControllerAction.h>
#include <math.h>

ControllerActionServer::ControllerActionServer(std::string name) :
as_(nh_, name, boost::bind(&ControllerActionServer::executeCB, this, _1), false),
    action_name_(name)
{
	//register the goal and feeback callbacks
	//as_.registerGoalCallback(boost::bind(&goalCB));
	//as_.registerPreemptCallback(boost::bind(&preemptCB, this));

	_forward_input = 0.0;
	_sidemove_input = 0.0;
	_heading_input = 0.0;
	_depth_input = 0.0;

	MIN_FORWARD = 0.10;
	MIN_SIDEMOVE = 0.10;
	MIN_HEADING = 1.0;
	MIN_DEPTH = 0.01;
	as_.start();
}


void ControllerActionServer::executeCB(const bbauv_msgs::ControllerGoalConstPtr &goal)
{
	bool isForwardDone = false,isDepthDone = false, isHeadingDone = false, isSidemoveDone = false;
	// helper variables
	ros::Rate r(10);
	bool success = true;
	goal_.depth_setpoint =  goal->depth_setpoint;
	goal_.heading_setpoint =  goal->heading_setpoint;
	goal_.forward_setpoint =  goal->forward_setpoint + _forward_input;
	goal_.sidemove_setpoint =  goal->sidemove_setpoint + _sidemove_input;
	// push_back the seeds for the fibonacci sequence
	//feedback_.forward_error = 1000;

	ROS_INFO("Action Server Goal received - f: %f, s: %f,h: %f, d: %f" , goal_.forward_setpoint,
			goal_.sidemove_setpoint, goal_.heading_setpoint,goal_.depth_setpoint);
	while(ros::ok() && success && (!isForwardDone || !isDepthDone || !isHeadingDone || !isSidemoveDone))
	{
		// publish info to the console for the user
		//ROS_INFO("error: %f", _forward_input);

		// check that preempt has not been requested by the client
		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
			goal_.forward_setpoint = _forward_input;
			goal_.depth_setpoint = _depth_input;
			goal_.heading_setpoint = _heading_input;
			goal_.sidemove_setpoint = _sidemove_input;
			success = false;
		}

		if(fabs(goal_.forward_setpoint - _forward_input) < MIN_FORWARD)
		{
			isForwardDone = true;
			ROS_DEBUG("isForwardDone");
		}

		if(fabs(goal_.sidemove_setpoint - _sidemove_input) < MIN_SIDEMOVE)
		{
			isSidemoveDone = true;
			ROS_DEBUG("isSidemoveDone");
		}

		if(fabs(goal_.depth_setpoint - _depth_input) < MIN_DEPTH)
		{
			isDepthDone = true;
			ROS_DEBUG("isDepthDone");
		}

		if(fabs(goal_.heading_setpoint - _heading_input) < MIN_HEADING)
		{
			isHeadingDone = true;
			ROS_DEBUG("isHeadingDone");
		}
		//Update Feedback
		feedback_.forward_error = fabs(goal_.forward_setpoint - _forward_input);
		feedback_.depth_error =fabs(goal_.depth_setpoint - _depth_input) ;
		feedback_.sidemove_error = fabs(goal_.sidemove_setpoint - _sidemove_input) ;
		feedback_.heading_error = fabs(goal_.heading_setpoint - _heading_input);
 		//publish the feedback
		as_.publishFeedback(feedback_);
		// this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		r.sleep();
	}

	 if(success)
	    {
			result_.forward_final = _forward_input;
			result_.sidemove_final = _sidemove_input;
			result_.heading_final = _heading_input;
			result_.depth_final = _depth_input;

		 	ROS_INFO("%s: Succeeded", action_name_.c_str());
		 	// set the action state to succeeded
		 	as_.setSucceeded(result_);

	    } else
	    {
	    	//ROS_INFO("%s: Aborted", action_name_.c_str());
	    	//as_.setAborted(result_);
	    }
}

void ControllerActionServer::updateState(float forward,float sidemove,float heading,float depth)
{
	_forward_input = forward;
	_sidemove_input = sidemove;
	_heading_input =  heading;
	_depth_input = depth;
}

//****************Getter Functions******************
float ControllerActionServer::getForward()
{
	return goal_.forward_setpoint;
}
float ControllerActionServer::getSidemove()
{
	return goal_.sidemove_setpoint;
}
float ControllerActionServer::getHeading()
{
	return goal_.heading_setpoint;
}
float ControllerActionServer::getDepth()
{
	return goal_.depth_setpoint;
}

ControllerActionServer::~ControllerActionServer() {
	// TODO Auto-generated destructor stub
}

