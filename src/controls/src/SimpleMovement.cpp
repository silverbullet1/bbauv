/* 
	SimpleMovement.cpp
	High level, simple movements layer for robot
	Date created: Jan 2014
	Author: Jason Poh
*/

#include "ros/ros.h"
#include "bbauv_msgs/SimpleMovement.h"

/*
	Simple Movements functions
*/

bool doYaw(int val)
{
	ROS_INFO("SimpleMovement: Changing yaw by %d", val);
}

bool doRoll(int val)
{
	ROS_INFO("SimpleMovement: Changing roll by %d", val);
}

bool doPitch(int val)
{
	ROS_INFO("SimpleMovement: Changing pitch by %d", val);
}

bool goToDepth(int metres)
{
	ROS_INFO("SimpleMovement: Going to %d (m) deep", metres);
}

bool moveLeft(int metres)
{
	ROS_INFO("SimpleMovement: Moving left by %d metres", metres);
}

bool moveRight(int metres)
{
	ROS_INFO("SimpleMovement: Moving right by %d metres", metres);
}

bool rotateRight(int degrees)
{
	ROS_INFO("SimpleMovement: Rotating right by %d degrees", degrees);
}

bool rotateLeft(int degrees)
{
	ROS_INFO("SimpleMovement: Rotating left by %d degrees", degrees);
}

/*
	Service required functions
*/

bool doYawSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	doYaw(val);
	res.isCompleted = true;
	return true;
}

bool doRollSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	doRoll(val);
	res.isCompleted = true;
	return true;
}

bool doPitchSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	doPitch(val);
	res.isCompleted = true;
	return true;
}

bool goToDepthSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	goToDepth(val);
	res.isCompleted = true;
	return true;
}

bool moveLeftSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	moveLeft(val);
	res.isCompleted = true;
	return true;
}

bool moveRightSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	moveRight(val);
	res.isCompleted = true;
	return true;
}

bool rotateLeftSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	rotateLeft(val);
	res.isCompleted = true;
	return true;
}

bool rotateRightSvc(bbauv_msgs::SimpleMovement::Request  &req,
         			bbauv_msgs::SimpleMovement::Response &res)
{
	int val = req.val;
	rotateRight(val);
	res.isCompleted = true;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_movement");
	ros::NodeHandle node;


	/*
		Initialize SimpleMovement services
	*/
	ros::ServiceServer svc_doYaw = node.advertiseService("simple_movement/doYaw", doYawSvc);
	ros::ServiceServer svc_doRoll = node.advertiseService("simple_movement/doRoll", doRollSvc);
	ros::ServiceServer svc_doPitch = node.advertiseService("simple_movement/doPitch", doPitchSvc);
	ros::ServiceServer svc_goToDepth = node.advertiseService("simple_movement/goToDepth", goToDepthSvc);
	ros::ServiceServer svc_moveLeft = node.advertiseService("simple_movement/moveLeft", moveLeftSvc);
	ros::ServiceServer svc_moveRight = node.advertiseService("simple_movement/moveRight", moveRightSvc);
	ros::ServiceServer svc_rotateLeft = node.advertiseService("simple_movement/rotateLeft", rotateLeftSvc);
	ros::ServiceServer svc_rotateRight = node.advertiseService("simple_movement/rotateRight", rotateRightSvc);

	while(ros::ok())
	{
		ros::AsyncSpinner spinner(16); // Use 16 threads
		spinner.start();
		ros::waitForShutdown();
	}
	return 0;
}
