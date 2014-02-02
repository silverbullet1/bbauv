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
#include <controls/PID_ControllerConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <PID_Controller/PID.h>
#include <NavUtils/NavUtils.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <bbauv_msgs/ControllerAction.h>
#include <ControllerActionServer/ControllerActionServer.h>
#include <geometry_msgs/Twist.h>
#include <bbauv_msgs/imu_data.h>
#include <bbauv_msgs/set_controller.h>
#include <bbauv_msgs/pid_info.h>
#include <bbauv_msgs/locomotion_mode.h>
#define _USE_MATH_DEFINES // for C++
#include <math.h>

const static int loop_frequency = 20;
const static int PSI30 = 206842;
const static int PSI100 = 689475;
const static int ATM = 99974; //Pascals or 14.5PSI

double thruster3_ratio =1,thruster4_ratio = 1,thruster5_ratio,thruster6_ratio;

bbauv_msgs::controller ctrl;
bbauv_msgs::thruster thrusterSpeed;
bbauv_msgs::depth depthReading;
bbauv_msgs::compass_data orientationAngles;
bbauv_msgs::pid_info pidInfo;
nav_msgs::Odometry odom_data;
double depth_offset = 0;

//State Machines
bool inTopside,inTeleop,inHovermode = false, oldHovermode = false;
bool inDepthPID, inHeadingPID, inForwardPID, inSidemovePID, inPitchPID, inRollPID;
bool inNavigation;
bool inVisionTracking;
bool isForward = false;
bool isSidemove = false;

/**********************Function Prototypes**********************************/
void collectPosition(const nav_msgs::Odometry::ConstPtr& msg);
void collectOrientation(const bbauv_msgs::imu_data::ConstPtr& msg);
void collectPressure(const std_msgs::Int16& msg);
void collectTeleop(const bbauv_msgs::thruster& msg);
void collectAutonomous(const bbauv_msgs::controller & msg);
void callback(PID_Controller::PID_ControllerConfig &config, uint32_t level);
double getHeadingPIDUpdate();

//Need to change these two functions for new configuration
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output);
void setVertThrustSpeed(double depthPID_output,double pitchPID_output,double rollPID_output);

double fmap(int input, int in_min, int in_max, int out_min, int out_max);
/**********************Publisher**********************************/
ros::Publisher thrusterPub;
ros::Publisher depthPub;
ros::Publisher orientationPub;
ros::Publisher controllerPub;
ros::Publisher locomotionModePub;
ros::Publisher pid_infoPub;
/**********************Subscriber**********************************/
ros::Subscriber orientationSub;
ros::Subscriber pressureSub;
ros::Subscriber teleopSub;
ros::Subscriber autonomousSub;
ros::Subscriber velocitySub;
/**********************PID Controllers**********************************/
bbauv::bbPID forwardPID("f",1.2,0,0,20);
bbauv::bbPID depthPID("d",1.2,0,0,20);
bbauv::bbPID headingPID("h",1.2,0,0,20);
bbauv::bbPID sidemovePID("s",1.2,0,0,20);
bbauv::bbPID pitchPID("p",1.2,0,0,20);
bbauv::bbPID rollPID("r",1.2,0,0,20);

int act_forward[2];
int act_sidemove[2];
int act_heading[2];

//Forward mode settings: Index 0 and 1
//Sidemove mode settings: Index 2 and 3
int loc_mode_forward[4] = {-2400,2400,-400,400};
int loc_mode_sidemove[4] = {-400,400,-2400,2400};
int loc_mode_heading[4] = {-400,400,-400,400};

int manual_speed[6] = {0,0,0,0,0,0};

bool locomotion_srv_handler(bbauv_msgs::locomotion_mode::Request  &req,
        bbauv_msgs::locomotion_mode::Response &res)
{
	int mode = 0;
	if(req.forward && req.sidemove)
	{
		res.success = false;
		return true;
	}
	else if(req.forward)
	{
		headingPID.setActuatorSatModel(loc_mode_heading[0],loc_mode_heading[1]);
		sidemovePID.setActuatorSatModel(loc_mode_sidemove[0],loc_mode_sidemove[1]);
		forwardPID.setActuatorSatModel(loc_mode_forward[0],loc_mode_forward[1]);
		res.success = true;
		mode = 1;
		ROS_INFO("Switching to Forward mode.");
	}else if(req.sidemove)
	{
		mode = 2;
		headingPID.setActuatorSatModel(loc_mode_heading[2],loc_mode_heading[3]);
		sidemovePID.setActuatorSatModel(loc_mode_sidemove[2],loc_mode_sidemove[3]);
		forwardPID.setActuatorSatModel(loc_mode_forward[2],loc_mode_forward[3]);
		ROS_INFO("Switching to Sidemove mode.");
		res.success = true;
	}else if(!req.forward && !req.sidemove)
	{
		mode = 0;
		ROS_DEBUG("h_min: %i,h_max: %i,s_min:%i,s_max:%i,f_min: %i,f_max: %i",act_heading[0],act_heading[1],act_sidemove[0],act_sidemove[1],act_forward[0],act_forward[1]);
		//If forward and sidemove mode are not activated, revert to default
		headingPID.setActuatorSatModel(act_heading[0],act_heading[1]);
		sidemovePID.setActuatorSatModel(act_sidemove[0],act_sidemove[1]);
		forwardPID.setActuatorSatModel(act_forward[0],act_forward[1]);
		ROS_INFO("Switching to Default mode.");
		res.success = true;
	} else	res.success = false;
	std_msgs::Int8 std_mode;
	std_mode.data = mode;
	locomotionModePub.publish(std_mode);
	return true;
}
bool controller_srv_handler(bbauv_msgs::set_controller::Request  &req,
         bbauv_msgs::set_controller::Response &res)
{
  inDepthPID = req.depth;
  inForwardPID = req.forward;
  inHeadingPID = req.heading;
  inSidemovePID = req.sidemove;
  inPitchPID = req.pitch;
  inRollPID = req.roll;
  inTopside = req.topside;
  inNavigation = req.navigation;
  res.complete = true;
  return true;
}

int main(int argc, char **argv)
{
	//Initialize PID output variables

	double forwardPIDoutput, headingPID_output,depthPID_output,sidemovePID_output,pitchPID_output, rollPID_output;

	//Initialize Node

	ros::init(argc, argv, "Controller");
	ros::NodeHandle nh;
	//Initialize Publishers

	thrusterPub = nh.advertise<bbauv_msgs::thruster>("/thruster_speed", 1000);
	depthPub = nh.advertise<bbauv_msgs::depth>("/depth",1000);
	orientationPub = nh.advertise<bbauv_msgs::compass_data>("/euler",1000);
	controllerPub = nh.advertise<bbauv_msgs::controller>("/controller_points",100);
	locomotionModePub = nh.advertise<std_msgs::Int8>("/locomotion_mode",100,true);
	pid_infoPub = nh.advertise<bbauv_msgs::pid_info>("/pid_info",1000);

	//Initialize Subscribers
	autonomousSub = nh.subscribe("/cmd_position",1000,collectAutonomous);
	velocitySub = nh.subscribe("/WH_DVL_data",1000,collectPosition);
	orientationSub = nh.subscribe("/AHRS8_data_e",1000,collectOrientation);
	pressureSub = nh.subscribe("/pressure_data",1000,collectPressure);
	teleopSub = nh.subscribe("/teleop_controller",1000,collectTeleop);
	dynamic_reconfigure::Server <PID_Controller::PID_ControllerConfig> server;
	dynamic_reconfigure::Server<PID_Controller::PID_ControllerConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	// Initialize Services

	ros::ServiceServer service = nh.advertiseService("set_controller_srv", controller_srv_handler);
	ROS_INFO("set_controller_srv ready.");

	ros::ServiceServer locomotion_service = nh.advertiseService("locomotion_mode_srv", locomotion_srv_handler);
	ROS_INFO("locomotion_mode_srv ready.");

	/* Initialize Action Server */

	ControllerActionServer as("LocomotionServer");
	//Execute PID Loop computation at 20Hz
	ros::Rate loop_rate(loop_frequency);

	//Initialize initial Locomotion Mode publish
	std_msgs::Int8 std_mode;
	std_mode.data = 0;
	locomotionModePub.publish(std_mode);
	ROS_INFO("PID Controllers Initialized. yay!");
	//PID Loop Computation
	//Loop running at 20Hz
	while(ros::ok())
	{
		/* To enable PID
		  Autonomous Control only if not in Topside state*/
		if(inHovermode && inTopside && oldHovermode != inHovermode)
		{
			ctrl.forward_setpoint = ctrl.forward_input;
			ctrl.sidemove_setpoint = ctrl.sidemove_input;
			ctrl.heading_setpoint = ctrl.heading_input;
			oldHovermode = inHovermode;
		}
		if(inHeadingPID)
		{
			headingPID_output = getHeadingPIDUpdate();
			pidInfo.heading.p = headingPID.getProportional();
			pidInfo.heading.i = headingPID.getIntegral();
			pidInfo.heading.d = headingPID.getDerivative();
			pidInfo.heading.total = headingPID.getTotal();
		}
		else
		{

			headingPID_output = 0;
			headingPID.clearIntegrator();
			}
		if(inDepthPID)
		{
			depthPID_output = depthPID.computePID((double)ctrl.depth_setpoint,ctrl.depth_input);
			pidInfo.depth.p = depthPID.getProportional();
			pidInfo.depth.i = depthPID.getIntegral();
			pidInfo.depth.d = depthPID.getDerivative();
			pidInfo.depth.total = depthPID.getTotal();
		}
		else
		{
			depthPID_output = 0;
			depthPID.clearIntegrator();
		}
		if(inForwardPID)
		{
			forwardPIDoutput = forwardPID.computePID(ctrl.forward_setpoint,ctrl.forward_input);
			pidInfo.forward.p = forwardPID.getProportional();
			pidInfo.forward.i = forwardPID.getIntegral();
			pidInfo.forward.d = forwardPID.getDerivative();
			pidInfo.forward.total = forwardPID.getTotal();
		}
		else
		{
			forwardPIDoutput = 0;
			forwardPID.clearIntegrator();
			}
		if(inSidemovePID)
		{
			pidInfo.sidemove.p = sidemovePID.getProportional();
			pidInfo.sidemove.i = sidemovePID.getIntegral();
			pidInfo.sidemove.d = sidemovePID.getDerivative();
			pidInfo.sidemove.total = sidemovePID.getTotal();
			sidemovePID_output = sidemovePID.computePID(ctrl.sidemove_setpoint,ctrl.sidemove_input);
		}
		else
		{
			sidemovePID_output = 0;
			sidemovePID.clearIntegrator();
		}
		if(inPitchPID)
		{
			pidInfo.pitch.p = pitchPID.getProportional();
			pidInfo.pitch.i = pitchPID.getIntegral();
			pidInfo.pitch.d = pitchPID.getDerivative();
			pidInfo.pitch.total = pitchPID.getTotal();
			pitchPID_output = pitchPID.computePID(ctrl.pitch_setpoint,ctrl.pitch_input);
		}
		else
		{
			pitchPID_output = 0;
			pitchPID.clearIntegrator();
		}
		if(inRollPID)
		{
			pidInfo.roll.p = rollPID.getProportional();
			pidInfo.roll.i = rollPID.getIntegral();
			pidInfo.roll.d = rollPID.getDerivative();
			pidInfo.roll.total = rollPID.getTotal();
			rollPID_output = rollPID.computePID(ctrl.roll_setpoint,ctrl.roll_input);
		}
		else
		{
			rollPID_output = 0;
			rollPID.clearIntegrator();
		}
		
		//need to modify these two lines
		setHorizThrustSpeed(headingPID_output,forwardPIDoutput,sidemovePID_output);
		setVertThrustSpeed(depthPID_output,pitchPID_output,rollPID_output);

		/*Update Action Server Positions*/
		if(!inNavigation && !inTopside)
		{
			ctrl.forward_setpoint = as.getForward();
			ctrl.sidemove_setpoint = as.getSidemove();
			ctrl.heading_setpoint = as.getHeading();
			ctrl.depth_setpoint = as.getDepth();
			as.updateState(ctrl.forward_input,ctrl.sidemove_input,ctrl.heading_input,ctrl.depth_input);
		}

		controllerPub.publish(ctrl);
		pid_infoPub.publish(pidInfo);
		thrusterPub.publish(thrusterSpeed);
		ROS_DEBUG(" F %i, SM%i, H %i, P %i, R %i, D %i, Nav %i",inForwardPID,inSidemovePID,inHeadingPID,inPitchPID, inRollPID, inDepthPID,inNavigation);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

double getHeadingPIDUpdate()
{
	double wrappedHeading;
	double error = (double) ctrl.heading_setpoint - (double) ctrl.heading_input;
	//Fix wrap around for angles
	wrappedHeading = headingPID.wrapAngle360(error,(double) ctrl.heading_input);
	//cout<<"yaw: "<<ctrl.heading_input<<" e: "<<error<<" hd: "<<wrappedHeading<<endl;
	ROS_INFO("val: %f, set: %f, err: %f",wrappedHeading, ctrl.heading_input,error);
	return headingPID.computePID(ctrl.heading_setpoint,wrappedHeading);
}
void setHorizThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output)
    {
      thrusterSpeed.speed1=forwardPID_output;
      thrusterSpeed.speed2=forwardPID_output;
      thrusterSpeed.speed7= -(double)headingPID_output-(double)sidemovePID_output;
      thrusterSpeed.speed8= (double)headingPID_output-(double)sidemovePID_output;
      //ROS_INFO("ts8: %d, ts7: %d", thrusterSpeed.speed8,thrusterSpeed.speed7);
      //ROS_INFO("o:%d",-(double)headingPID_output-(double)sidemovePID_output);
    }

void setVertThrustSpeed(double depthPID_output,double pitchPID_output, double rollPID_output)
  {
	double speed3_output = thruster3_ratio*(- depthPID_output + pitchPID_output - rollPID_output);
	double speed4_output = thruster4_ratio*(- depthPID_output + pitchPID_output + rollPID_output);
	double speed5_output = thruster5_ratio*(- depthPID_output - pitchPID_output + rollPID_output);
	double speed6_output = thruster6_ratio*(- depthPID_output - pitchPID_output - rollPID_output);

    if(speed3_output < -3200) thrusterSpeed.speed3 = -3200;
    else if(speed3_output >3200) thrusterSpeed.speed3 = 3200;
    else thrusterSpeed.speed3= speed3_output;

    if(speed4_output < -3200) thrusterSpeed.speed4 = -3200;
    else if(speed4_output >3200) thrusterSpeed.speed4 = 3200;
    else thrusterSpeed.speed4= speed4_output;

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
void collectOrientation(const bbauv_msgs::imu_data::ConstPtr& msg)
{
	ctrl.heading_input =  msg->orientation.z*180/M_PI;
	ctrl.pitch_input = msg->orientation.y*180/M_PI;
        ctrl.roll_input = msg->orientation.x*180/M_PI;
	orientationAngles.roll = ctrl.roll_input;
	orientationAngles.yaw = ctrl.heading_input;
	orientationAngles.pitch = ctrl.pitch_input;
	orientationPub.publish(orientationAngles);
}

void collectPressure(const std_msgs::Int16& msg)
{
	//Case for Current Loop Sensor shield
	//double pressure = 3*(double) msg.data/2048 - 7.5; //In PSI

	// Case for Adafruit ADC raw current loop sensing
	double pressure = 15*(double) msg.data/10684 - 7.498596031;

	pressure*= 6895; //Convert to Pascals
	double depth = pressure/(1000*9.81) - depth_offset;
	ctrl.depth_input = depth;
	depthReading.depth = depth;
	depthReading.pressure = pressure + ATM;
	depthPub.publish(depthReading);
}
void collectPosition(const nav_msgs::Odometry::ConstPtr& msg)
{
		ctrl.forward_input = msg->pose.pose.position.x;
		ctrl.sidemove_input = msg->pose.pose.position.y;
}

/*****************Helper Functions*********************/
//Remap ADC 16 bit from 4mA to 20mA to the pressure range
double fmap(int input, int in_min, int in_max, int out_min, int out_max){
  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//no longer in use
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

void collectAutonomous(const bbauv_msgs::controller & msg)
{
	if(inNavigation)
	{
		ctrl.forward_setpoint = msg.forward_setpoint;
		ctrl.sidemove_setpoint = msg.sidemove_setpoint;
		ctrl.heading_setpoint = msg.heading_setpoint;
	}
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
  inRollPID = config.roll_PID;
  inNavigation = config.navigation;
  inHovermode = config.hovermode;
  thruster5_ratio = config.thruster5_ratio;
  thruster6_ratio = config.thruster6_ratio;
  thruster3_ratio = config.thruster3_ratio;
  thruster4_ratio = config.thruster4_ratio;


  ctrl.heading_setpoint = config.heading_setpoint;
  ctrl.depth_setpoint = config.depth_setpoint;
  ctrl.sidemove_setpoint = config.sidemove_setpoint;
  ctrl.forward_setpoint = config.forward_setpoint;
  ctrl.pitch_setpoint = config.pitch_setpoint;
  ctrl.roll_setpoint = config.roll_setpoint;

  depthPID.setKp(config.depth_Kp);
  depthPID.setTi(config.depth_Ti);
  depthPID.setTd(config.depth_Td);
  depth_offset = config.depth_offset;
  depthPID.setActuatorSatModel(config.depth_min,config.depth_max);

  headingPID.setKp(config.heading_Kp);
  headingPID.setTi(config.heading_Ti);
  headingPID.setTd(config.heading_Td);
  headingPID.setActuatorSatModel(config.heading_min,config.heading_max);
  act_heading[0] = config.heading_min;
  act_heading[1] = config.heading_max;

  forwardPID.setKp(config.forward_Kp);
  forwardPID.setTi(config.forward_Ti);
  forwardPID.setTd(config.forward_Td);
  forwardPID.setActuatorSatModel(config.forward_min,config.forward_max);
  act_forward[0] = config.forward_min;
  act_forward[1] = config.forward_max;

  sidemovePID.setKp(config.sidemove_Kp);
  sidemovePID.setTi(config.sidemove_Ti);
  sidemovePID.setTd(config.sidemove_Td);
  sidemovePID.setActuatorSatModel(config.sidemove_min,config.sidemove_max);
  act_sidemove[0] = config.sidemove_min;
  act_sidemove[1] = config.sidemove_max;

  pitchPID.setKp(config.pitch_Kp);
  pitchPID.setTd(config.pitch_Td);
  pitchPID.setTi(config.pitch_Ti);
  pitchPID.setActuatorSatModel(config.pitch_min,config.pitch_max);

  rollPID.setKp(config.roll_Kp);
  rollPID.setTd(config.roll_Td);
  rollPID.setTi(config.roll_Ti);
  rollPID.setActuatorSatModel(config.roll_min,config.roll_max);

  }

