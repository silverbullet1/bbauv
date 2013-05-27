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
#include <std_msgs/Int16.h>
#include <PID_Controller/PID.h>
#include <NavUtils/NavUtils.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <PID_Controller/ControllerAction.h>
#include <ControllerActionServer/ControllerActionServer.h>
#include <geometry_msgs/Twist.h>

const static int loop_frequency = 20;
const static int PSI30 = 206842;
const static int PSI100 = 689475;
const static int ATM = 99974; //Pascals or 14.5PSI

bbauv_msgs::controller ctrl;
bbauv_msgs::thruster thrusterSpeed;
bbauv_msgs::depth depthReading;
bbauv_msgs::compass_data orientationAngles;
nav_msgs::Odometry odom_data;
double depth_offset = 0;

//State Machines
bool inTopside,inTeleop;
bool inDepthPID, inHeadingPID, inForwardPID, inSidemovePID,inPitchPID;
bool inNavigation;
bool inVisionTracking;

/**********************Function Prototypes**********************************/
void collectVelocity(const nav_msgs::Odometry::ConstPtr& msg);
void collectOrientation(const sensor_msgs::Imu::ConstPtr& msg);
void collectPressure(const std_msgs::Int16& msg);
void collectTeleop(const bbauv_msgs::thruster& msg);
void collectAutonomous(const geometry_msgs::Twist & msg);
void callback(PID_Controller::PID_ControllerConfig &config, uint32_t level);
double getHeadingPIDUpdate();
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output);
void setVertThrustSpeed(double depthPID_output,double pitchPID_output);
double fmap(int input, int in_min, int in_max, int out_min, int out_max);
/**********************Publisher**********************************/
ros::Publisher thrusterPub;
ros::Publisher depthPub;
ros::Publisher orientationPub;
/**********************Subscriber**********************************/
ros::Subscriber orientationSub;
ros::Subscriber pressureSub;
ros::Subscriber teleopSub;
ros::Subscriber autonomousSub;
ros::Subscriber velocitySub;

/**********************Action Server**********************************/



/**********************PID Controllers**********************************/
bbauv::bbPID forwardPID("f",1.2,0,0,20);
bbauv::bbPID depthPID("d",1.2,0,0,20);
bbauv::bbPID headingPID("h",1.2,0,0,20);
bbauv::bbPID sidemovePID("s",1.2,0,0,20);
bbauv::bbPID pitchPID("p",1.2,0,0,20);

NavUtils navHelper;

int manual_speed[6] = {0,0,0,0,0,0};

int main(int argc, char **argv)
{
	//Initialize PID output variables

	double forwardPIDoutput, headingPID_output,depthPID_output,sidemovePID_output,pitchPID_output;

	//Initialize Node

	ros::init(argc, argv, "Controller");
	ros::NodeHandle nh;
	//Initialize Publishers

	thrusterPub = nh.advertise<bbauv_msgs::thruster>("/thruster_speed", 1000);
	depthPub = nh.advertise<bbauv_msgs::depth>("/depth",1000);
	orientationPub = nh.advertise<bbauv_msgs::compass_data>("/euler",1000);

	//Initialize Subscribers

	autonomousSub = nh.subscribe("/cmd_val",1000,collectAutonomous);
	velocitySub = nh.subscribe("/WH_DVL_data",1000,collectVelocity);
	orientationSub = nh.subscribe("/AHRS8_data",1000,collectOrientation);
	pressureSub = nh.subscribe("/pressure_data",1000,collectPressure);
	teleopSub = nh.subscribe("/teleop_controller",1000,collectTeleop);
	dynamic_reconfigure::Server <PID_Controller::PID_ControllerConfig> server;
	dynamic_reconfigure::Server<PID_Controller::PID_ControllerConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	/* Initialize Quaternion Conversion Helper*/

	navHelper = NavUtils();

	/* Initialize Action Server */

	ControllerActionServer as("LocomotionServer");
	//Execute PID Loop computation at 20Hz
	ros::Rate loop_rate(loop_frequency);

	ROS_INFO("PID Controllers Initialized.");
	//PID Initialization
	while(ros::ok())
	{
		/* To enable PID
		  Autonomous Control only if not in Topside state*/
		if(inNavigation)
		if(inHeadingPID)	headingPID_output = getHeadingPIDUpdate();
		else headingPID_output = 0;
		if(inDepthPID)		depthPID_output = depthPID.computePID((double)ctrl.depth_setpoint,ctrl.depth_input);
		else depthPID_output = 0;
		if(inForwardPID)	forwardPIDoutput = forwardPID.computePID(ctrl.forward_setpoint,ctrl.forward_input);
		else forwardPIDoutput = 0;
		if(inSidemovePID)	sidemovePID_output = sidemovePID.computePID(ctrl.sidemove_setpoint,ctrl.sidemove_input);
		else sidemovePID_output = 0;
		if(inPitchPID) pitchPID_output = pitchPID.computePID(ctrl.pitch_setpoint,ctrl.pitch_input);
		else pitchPID_output = 0;
		setHorizThrustSpeed(headingPID_output,forwardPIDoutput,sidemovePID_output);
		setVertThrustSpeed(depthPID_output,pitchPID_output);

		/*Update Action Server Positions*/
		ctrl.forward_setpoint = as.getForward();
		ctrl.sidemove_setpoint = as.getSidemove();
		ctrl.heading_setpoint = as.getHeading();
		ctrl.depth_setpoint = as.getDepth();

		as.updateState(ctrl.forward_input,ctrl.sidemove_input,ctrl.heading_input,ctrl.depth_input);

		thrusterPub.publish(thrusterSpeed);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

double getHeadingPIDUpdate()
{
	double wrappedHeading;
	double error = (double) ctrl.heading_setpoint - ctrl.heading_input;
	//Fix wrap around for angles
	wrappedHeading = headingPID.wrapAngle360(error,ctrl.heading_input);
	//cout<<"yaw: "<<ctrl.heading_input<<" e: "<<error<<" hd: "<<wrappedHeading<<endl;
	return headingPID.computePID(ctrl.heading_setpoint,wrappedHeading);
}
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output)
    {
      thrusterSpeed.speed1=-headingPID_output-forwardPID_output-sidemovePID_output + manual_speed[0];
      thrusterSpeed.speed2=headingPID_output+forwardPID_output-sidemovePID_output + manual_speed[1];
      thrusterSpeed.speed3=headingPID_output-forwardPID_output+sidemovePID_output + manual_speed[2];
      thrusterSpeed.speed4=-headingPID_output+forwardPID_output+sidemovePID_output + manual_speed[3];
    }

void setVertThrustSpeed(double depthPID_output,double pitchPID_output)
  {
	double speed5_output = - depthPID_output + pitchPID_output + manual_speed[4];
	double speed6_output = - depthPID_output - pitchPID_output + manual_speed[5];
    if(speed5_output < -3200) thrusterSpeed.speed5 = -3200;
    else if(speed5_output >3200) thrusterSpeed.speed5 = 3200;
    else thrusterSpeed.speed5= speed5_output;

    if(speed6_output < -3200) thrusterSpeed.speed6 = -3200;
    else if(speed6_output >3200) thrusterSpeed.speed6 = 3200;
    else thrusterSpeed.speed6= speed6_output;
  }

/***********Subscriber Callbacks*****************/

void collectInput(const bbauv_msgs::controller &msg)
{

}
void collectOrientation(const sensor_msgs::Imu::ConstPtr& msg)
{
	double q0 = (msg->orientation).w;
	double q1 = (msg->orientation).x;
	double q2 = (msg->orientation).y;
	double q3 = (msg->orientation).z;

	ctrl.heading_input =  360 - (navHelper.quaternionToYaw(q0,q1,q2,q3) + 180);
	ctrl.pitch_input = 360 - (navHelper.quaternionToPitch(q0,q1,q2,q3) + 180);
	orientationAngles.yaw = ctrl.heading_input;
	orientationAngles.pitch = ctrl.pitch_input;
	orientationPub.publish(orientationAngles);
	//cout<<ctrl.pitch_input<<endl;
}

void collectPressure(const std_msgs::Int16& msg)
{
	//double pressure = fmap(msg.data, 5340,26698,0,PSI30);
	double pressure = 3*(double) msg.data/2048 - 7.5; //In PSI
	pressure*= 6895; //Convert to Pascals
	double depth = pressure/(1000*9.81) - depth_offset;
	ctrl.depth_input = depth;
	depthReading.depth = depth;
	depthReading.pressure = pressure;
	depthPub.publish(depthReading);
}
void collectVelocity(const nav_msgs::Odometry::ConstPtr& msg)
{
	ctrl.forward_input = msg->pose.pose.position.x;
	ctrl.sidemove_input = msg->pose.pose.position.y;
}

/*****************Helper Functions*********************/
//Remap ADC 16 bit from 4mA to 20mA to the pressure range
double fmap(int input, int in_min, int in_max, int out_min, int out_max){
  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void collectTeleop(const bbauv_msgs::thruster &msg)
{
	if(inTopside && inTeleop)
	{
		manual_speed[0] = msg.speed1;
		manual_speed[1] = msg.speed2;
		manual_speed[2] = msg.speed3;
		manual_speed[3] = msg.speed4;
		manual_speed[4] = msg.speed5;
		manual_speed[5] = msg.speed6;
	} else
	{
		manual_speed[0] = 0;
		manual_speed[1] = 0;
		manual_speed[2] = 0;
		manual_speed[3] = 0;
		manual_speed[4] = 0;
		manual_speed[5] = 0;
	}
}

void collectAutonomous(const geometry_msgs::Twist & msg)
{
	ctrl.forward_input = msg.linear.x;
	ctrl.sidemove_input = msg.linear.x;
}
/***********Dynamic Reconfigure Callbacks*****************/

void callback(PID_Controller::PID_ControllerConfig &config, uint32_t level) {
  inTopside = config.topside;
  inTeleop = config.teleop;
  inForwardPID = config.forward_PID;
  inHeadingPID = config.heading_PID;
  inDepthPID = config.depth_PID;
  inSidemovePID = config.sidemove_PID;
  inPitchPID = config.pitch_PID;

  ctrl.heading_setpoint = config.heading_setpoint;
  ctrl.depth_setpoint = config.depth_setpoint;
  ctrl.sidemove_setpoint = config.sidemove_setpoint;
  ctrl.forward_setpoint = config.forward_setpoint;
  ctrl.pitch_setpoint = config.pitch_setpoint;
  depthPID.setKp(config.depth_Kp);
  depthPID.setTi(config.depth_Ti);
  depthPID.setTd(config.depth_Td);
  depth_offset = config.depth_offset;
  depthPID.setActuatorSatModel(config.depth_min,config.depth_max);

  headingPID.setKp(config.heading_Kp);
  headingPID.setTi(config.heading_Ti);
  headingPID.setTd(config.heading_Td);
  headingPID.setActuatorSatModel(config.heading_min,config.heading_max);

  forwardPID.setKp(config.forward_Kp);
  forwardPID.setTi(config.forward_Ti);
  forwardPID.setTd(config.forward_Td);
  forwardPID.setActuatorSatModel(config.forward_min,config.forward_max);
  sidemovePID.setKp(config.sidemove_Kp);
  sidemovePID.setTi(config.sidemove_Ti);
  sidemovePID.setTd(config.sidemove_Td);
  sidemovePID.setActuatorSatModel(config.sidemove_min,config.sidemove_max);

  pitchPID.setKp(config.pitch_Kp);
  pitchPID.setTd(config.pitch_Td);
  pitchPID.setTi(config.pitch_Ti);
  pitchPID.setActuatorSatModel(config.pitch_min,config.pitch_max);
}

