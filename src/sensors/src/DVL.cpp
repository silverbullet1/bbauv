/*
 * dvl.cpp
 *	For Dead Reckoning
 *  Created on: 14 Jan, 2014
 *      Author: huixian
 */

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>

#include "bbauv_msgs/DVL.h"

struct position{
	int yaw, pitch, roll, forward, backward, depth, left, right, rotateLeft, rotateRight;
	double rel_xpos, rel_ypos, rel_zpos, rel_heading;
};

struct direction{
	double xpos, ypos, zpos;
};

bool updatePosition(bbauv_msgs::DVL::Request &req,
							bbauv_msgs::DVL::Response &res);

class DVL{
public:
	DVL();

	void reverseAngleVectors();
	direction addVectors(direction a, direction b, direction c);
	void calculateAngles();
	void findRobotHeading();
	void findRobotPositions();
	double returnDegrees(double angle);
	position positionalData;


private:
	ros::NodeHandle nh;

	//Subscribers to respective topics
	ros::Subscriber compassSub;
	ros::Subscriber thrusterSub;

	direction fwd, right, up;

};
DVL *dvl;

DVL::DVL(){
	ros::ServiceServer svc = nh.advertiseService("DVL/updatePosition", updatePosition);

	//Initialise the structure positionalData
	positionalData.rel_xpos = 0.0;
	positionalData.rel_ypos = 0.0;
	positionalData.rel_zpos = 0.0;
	positionalData.rel_heading = 0.0;
}

int kfd = 0;
struct termios cooked;

void quit(int sig){
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "dvl");
	ROS_INFO("Initialising DVL...");
	std::cout << M_PI << std::endl;


	DVL local_dvl;
	dvl = &local_dvl;
	signal(SIGINT, quit);

	while(ros::ok()){
		ros::AsyncSpinner spinner(20);
		spinner.start();
		ros::waitForShutdown();
	}

	return(0);
}

bool updatePosition(bbauv_msgs::DVL::Request &req,
							bbauv_msgs::DVL::Response &res){
	dvl->positionalData.yaw = req.yaw;
	dvl->positionalData.pitch = req.pitch;
	dvl->positionalData.roll = req.roll;
	dvl->positionalData.left = req.left;
	dvl->positionalData.right = req.right;
	dvl->positionalData.depth = req.depth;
	dvl->positionalData.forward = req.forward;
	dvl->positionalData.backward = req.backward;
	dvl->positionalData.rotateLeft = req.rotateLeft;
	dvl->positionalData.rotateRight = req.rotateRight;

	ROS_INFO("Inputing information...");

	dvl->calculateAngles();
	dvl->findRobotPositions();
	dvl->findRobotHeading();

	res.isCompleted = true;
	return true;
}

// Functions to update the values

//Calculate angles from Yaw Pitch Roll
void DVL::calculateAngles(){
	ROS_INFO("Calculating angles.... ");
	double anglePitch = positionalData.pitch * (M_PI*2 /360);
	double sinPitch = sin(anglePitch);
	double cosPitch = cos(anglePitch);

	double angleYaw = positionalData.yaw * (M_PI*2 / 360);
	double sinYaw = sin(angleYaw);
	double cosYaw = cos(angleYaw);

	double angleRoll = positionalData.roll * (M_PI*2 / 360);
	double sinRoll = sin(angleRoll);
	double cosRoll = cos(angleRoll);

	fwd.xpos = cosPitch * cosYaw;
	fwd.ypos = cosPitch * sinYaw;
	fwd.zpos = -sinPitch;

	right.xpos = (-1)*sinRoll*sinPitch*cosYaw + (-1)*cosRoll*(-cosYaw);
	right.ypos = (-1)*sinRoll*sinPitch*cosYaw + (-1)*cosRoll*cosYaw;
	right.zpos = (-1)*sinRoll*cosPitch;

	up.xpos = cosRoll*sinPitch*cosYaw + (-sinRoll*-sinYaw);
	up.ypos = cosRoll*sinPitch*sinYaw + (-sinRoll*cosYaw);
	up.zpos = cosRoll * cosPitch;

}

// To find relative positioning of robot
void DVL::findRobotPositions(){
	//positionalData.rel_xpos = addVectors(fwd)
}

// To find heading of robot
void DVL::findRobotHeading(){

}


// Helper function to Add vectors
direction DVL::addVectors(direction a, direction b, direction c){
	direction result;
	result.xpos = a.xpos + b.xpos + c.xpos;
	result.ypos = a.ypos + b.ypos + c.ypos;
	result.zpos = a.zpos + b.zpos + c.zpos;

	return result;
}


// If you want to reverse engineer the YPR from XYZ
double DVL::returnDegrees(double angle){
	return angle*180/M_PI;
}

void DVL::reverseAngleVectors(){
	double pitch = asin(-fwd.zpos);
	double yaw = returnDegrees(atan2(fwd.ypos, up.xpos));
	double roll = returnDegrees(atan2(-right.zpos,up.zpos));
}

