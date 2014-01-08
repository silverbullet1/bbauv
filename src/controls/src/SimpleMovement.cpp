/* 
	SimpleMovement.cpp
	High level, simple movements layer for robot
	Date created: Jan 2014
	Author: Jason Poh
*/

#include "ros/ros.h"
//#include "bbauv_msgs/TimeElapsed.h"

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_movement");
	ros::NodeHandle node;
	ros::Rate rate(30);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
