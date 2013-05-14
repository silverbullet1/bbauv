/*
 * PID_Controller.cpp
 *
 *  Created on: May 9, 2013
 *      Author: gohew
 */
#include "ros/ros.h"
#include "bbauv_msgs/thruster.h"
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/depth.h>
#include <bbauv_msgs/controller.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <PID_Controller/PID_ControllerConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <PID_Controller/PID.h>

using namespace std_msgs;
using namespace ros;
using namespace bbauv_msgs;
using namespace std;
using namespace bbauv;
using namespace PID_Controller;

const static int loop_frequency = 20;
controller ctrl;
thruster thrusterSpeed;
double depth_offset = 0;

//State Machines
bool inTopside,inTeleop;
bool inDepthPID, inHeadingPID, inForwardPID, inSidemovePID;
bool inNavigation;
bool inVisionTracking;

/**********************Function Prototypes**********************************/
//void collectVelocity(const nav_msgs::Odometry::ConstPtr& msg);
void collectOrientation(const compass_data& msg);
void collectDepth(const depth& msg);
void collectTeleop(const thruster& msg);
void callback(PID_ControllerConfig &config, uint32_t level);
double getHeadingPIDUpdate();
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output);
void setVertThrustSpeed(double depthPID_output,double pitchPID_output);
/**********************Publisher**********************************/
Publisher thrusterPub;
/**********************Subscriber**********************************/
Subscriber orientationSub;
Subscriber depthSub;
Subscriber teleopSub;

/**********************PID Controllers**********************************/
bbPID forwardPID(1.2,0,0,20);
bbPID depthPID(1.2,0,0,20);
bbPID headingPID(1.2,0,0,20);
bbPID sidemovePID(1.2,0,0,20);
bbPID pitchPID(1.2,0,0,20);

int manual_speed[6];

int main(int argc, char **argv)
{
	double forwardPIDoutput, headingPID_output,depthPID_output,sidemovePID_output,pitchPID_output;

	//Initialize Node
	init(argc, argv, "Controller");
	NodeHandle nh;
	//Initialize Publisher
	thrusterPub = nh.advertise<thruster>("/thruster_speed", 1000);
	orientationSub = nh.subscribe("/WH_DVL_data",100,collectOrientation);
	depthSub = nh.subscribe("/depth",100,collectDepth);
	teleopSub = nh.subscribe("/teleop_controller",100,collectTeleop);
	dynamic_reconfigure::Server<PID_ControllerConfig> server;
	dynamic_reconfigure::Server<PID_ControllerConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	//Execute PID Loop computation at 20Hz
	Rate loop_rate(loop_frequency);

	//PID Initialization
	while(ok())
	{
		/* To enable PID
		  Autonomous Control only if not in Topside state*/
		if(!inTeleop && !inTopside)
		{
			if(inHeadingPID)	headingPID_output = getHeadingPIDUpdate();
			if(inDepthPID)	depthPID_output = depthPID.computePID(ctrl.depth_setpoint,ctrl.depth_input);
			if(inForwardPID)	forwardPIDoutput = forwardPID.computePID(ctrl.forward_setpoint,ctrl.forward_input);
			if(inSidemovePID)	sidemovePID_output = sidemovePID.computePID(ctrl.sidemove_setpoint,ctrl.sidemove_input);
			setHorizThrustSpeed(headingPID_output,forwardPIDoutput,sidemovePID_output);
			setVertThrustSpeed(depthPID_output,pitchPID_output);
		}

		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

double getHeadingPIDUpdate()
{
	double wrappedHeading;
	double error = ctrl.heading_setpoint - ctrl.heading_input;
	//Fix wrap around for angles
	wrappedHeading = headingPID.wrapAngle360(error,ctrl.heading_input);
	return headingPID.computePID(ctrl.heading_setpoint,wrappedHeading);
}
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output)
    {
      thrusterSpeed.speed1=-headingPID_output-forwardPID_output-sidemovePID_output;
      thrusterSpeed.speed2=headingPID_output+forwardPID_output-sidemovePID_output;
      thrusterSpeed.speed3=headingPID_output-forwardPID_output+sidemovePID_output;
      thrusterSpeed.speed4=-headingPID_output+forwardPID_output+sidemovePID_output;
    }

void setVertThrustSpeed(double depthPID_output,double pitchPID_output)
  {
    thrusterSpeed.speed5= - depthPID_output + pitchPID_output;
    thrusterSpeed.speed6= - depthPID_output - pitchPID_output;
  }

/***********Subscriber Callbacks*****************/

void collectOrientation(const compass_data& msg)
{
	 ctrl.heading_input = msg.yaw;
	 ctrl.heading_input = msg.pitch;
}

void collectDepth(const depth& msg)
{
	ctrl.depth_input = msg.depth;
}

void collectTeleop(const thruster &msg)
{
	if(inTopside)
	{
		thrusterSpeed.speed1 = msg.speed1;
		thrusterSpeed.speed2 = msg.speed2;
		thrusterSpeed.speed3 = msg.speed3;
		thrusterSpeed.speed4 = msg.speed4;
		thrusterSpeed.speed5 = msg.speed5;
		thrusterSpeed.speed6 = msg.speed6;
		thrusterPub.publish(thrusterSpeed);
	}
}

/***********Dynamic Reconfigure Callbacks*****************/

void callback(PID_ControllerConfig &config, uint32_t level) {
  inTopside = config.topside;
  inTeleop = config.teleop;
  inForwardPID = config.forward_PID;
  inHeadingPID = config.heading_PID;
  inDepthPID = config.depth_PID;
  inSidemovePID = config.sidemove_PID;

  ctrl.heading_setpoint = config.heading_setpoint;
  ctrl.depth_setpoint = config.depth_setpoint;
  ctrl.sidemove_setpoint = config.sidemove_setpoint;
  ctrl.forward_setpoint = config.forward_setpoint;
  depthPID.setKp(config.depth_Kp);
  depthPID.setTi(config.depth_Ti);
  depthPID.setTd(config.depth_Td);
  headingPID.setKp(config.heading_Kp);
  headingPID.setTi(config.heading_Ti);
  headingPID.setTd(config.heading_Td);
  depth_offset = config.depth_offset;
}
