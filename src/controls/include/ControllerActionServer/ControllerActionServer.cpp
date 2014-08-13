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

	MIN_FORWARD = 0.05;
	MIN_SIDEMOVE = 0.05;
	MIN_FORWARD_VEL = 0.01;
	MIN_SIDEMOVE_VEL = 0.01;
	MIN_HEADING = 1;
	MIN_DEPTH = 0.02;
	as_.start();
}


void ControllerActionServer::executeCB(const bbauv_msgs::ControllerGoalConstPtr &goal)
{
	bool isForwardDone = false,isDepthDone = false, isHeadingDone = false, isSidemoveDone = false;
	// helper variables
	ros::Rate r(10);
	bool success = true;
	double yaw_error = 0;
	goal_.depth_setpoint =  goal->depth_setpoint;
	goal_.heading_setpoint =  goal->heading_setpoint;

        goal_.forward_setpoint =  goal->forward_setpoint + _forward_input;
        goal_.sidemove_setpoint =  goal->sidemove_setpoint + _sidemove_input;
        goal_.forward_vel_setpoint = goal->forward_vel_setpoint;
        goal_.sidemove_vel_setpoint = goal->sidemove_vel_setpoint;

	ROS_INFO("Action Server Goal received - f: %3.2f,f_g: %3.2f , s: %3.2f , s_g: %3.2f ,vf_g: %3.2f, vs_g: %3.2f, h: %3.2f, d: %3.3f" , goal_.forward_setpoint, goal->forward_setpoint,
			goal_.sidemove_setpoint,goal->sidemove_setpoint,goal->forward_vel_setpoint,goal->sidemove_vel_setpoint,goal_.heading_setpoint,goal_.depth_setpoint);
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
			goal_.forward_vel_setpoint = 0;
			goal_.sidemove_vel_setpoint = 0;
			success = false;
		}

		if(fabs(goal_.forward_setpoint - _forward_input) < MIN_FORWARD && isFwdPos)
		{
			isForwardDone = true;
			ROS_DEBUG("isForward Pos Done");
		}

		if(fabs(goal_.forward_vel_setpoint - _forward_vel_input) <MIN_FORWARD_VEL && isFwdVel)
		{
		        isForwardDone = true;
		        ROS_INFO("isForwardVel Pos Done");
		}

		if(fabs(goal_.sidemove_setpoint - _sidemove_input) < MIN_SIDEMOVE)
		{
			isSidemoveDone = true;
			ROS_DEBUG("isSidemoveDone");
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
		yaw_error = fabs(goal_.heading_setpoint - wrapAngle360(goal_.heading_setpoint,_heading_input));

		if(yaw_error < MIN_HEADING)
		{
			isHeadingDone = true;
			ROS_DEBUG("isHeadingDone");
		}
		//Update Feedback
		if(isFwdPos) feedback_.forward_error = fabs(goal_.forward_setpoint - _forward_input);
		else if(isFwdVel) feedback_.forward_error = fabs(goal_.forward_vel_setpoint - _forward_vel_input);
		feedback_.depth_error =fabs(goal_.depth_setpoint - _depth_input) ;
		feedback_.sidemove_error = fabs(goal_.sidemove_setpoint - _sidemove_input) ;
		feedback_.heading_error = yaw_error;
 		//publish the feedback
		as_.publishFeedback(feedback_);
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

void ControllerActionServer::updateState(float forward,float sidemove,float forward_vel, float sidemove_vel,float heading,float depth)
{
	_forward_input = forward;
	_sidemove_input = sidemove;
	_heading_input =  heading;
	_depth_input = depth;
	_forward_vel_input = forward_vel;
	_sidemove_vel_input = sidemove_vel;
}

double ControllerActionServer::wrapAngle360(double setpoint, double heading)
{
	double error = setpoint - heading;
	if (error > 180)
	{
		heading +=360;
	}
	else if (error < -180)
	{
		heading -= 360;
	}

	return fabs(heading);
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

float ControllerActionServer::getForwardVel()
{
        return goal_.forward_vel_setpoint;
}
float ControllerActionServer::getSidemoveVel()
{
        return goal_.sidemove_vel_setpoint;
}

float ControllerActionServer::getHeading()
{
	return goal_.heading_setpoint;
}
float ControllerActionServer::getDepth()
{
	return goal_.depth_setpoint;
}

void ControllerActionServer::setNavigation(bool nav)
{
  _inNavigation = nav;
}

void ControllerActionServer::setDispMode(bool isVelSide,bool isVelFwd)
{
  if(isVelSide)
  {
    isSideVel = true;
    isSidePos = false;
  }
  else
  {
    isSidePos = true;
    isSideVel = false;
  }

  if(isVelFwd)
  {
    isFwdVel = true;
    isFwdPos = false;
  }
  else
  {
    isFwdVel = false;
    isFwdPos = true;

  }
}

ControllerActionServer::~ControllerActionServer() {
  _inNavigation = false;
  isFwdPos = false;
  isFwdVel = false;
  isSidePos = false;
  isSideVel = false;
}

