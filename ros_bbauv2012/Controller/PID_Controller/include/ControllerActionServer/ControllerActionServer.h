/*
 * ControllerActionServer.h
 *
 *  Created on: 2013-05-27
 *      Author: gohew
 */

#ifndef CONTROLLERACTIONSERVER_H_
#define CONTROLLERACTIONSERVER_H_
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <PID_Controller/ControllerAction.h>


class ControllerActionServer {
public:
	ControllerActionServer(std::string name);
	void updateState(float forward,float sidemove,float heading,float depth);
	void executeCB(const PID_Controller::ControllerGoalConstPtr &goal);
	float getForward();
	float getSidemove();
	float getHeading();
	float getDepth();

	virtual ~ControllerActionServer();

private:
	float _forward_input;
	float _sidemove_input;
	float _heading_input;
	float _depth_input;
	float _forward_setpoint;
	float _sidemove_setpoint;
	float _heading_setpoint;
	float _depth_setpoint;

	float MIN_FORWARD;
	float MIN_SIDEMOVE;
	float MIN_HEADING;
	float MIN_DEPTH;

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<PID_Controller::ControllerAction> as_;
  std::string action_name_;
  int data_count_;
  float sum_, sum_sq_;
  PID_Controller::ControllerGoal goal_;
  PID_Controller::ControllerFeedback feedback_;
  PID_Controller::ControllerResult result_;
};

#endif /* CONTROLLERACTIONSERVER_H_ */