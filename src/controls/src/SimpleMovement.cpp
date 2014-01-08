/* 
	SimpleMovement.cpp
	High level, simple movements layer for robot
	Date created: Jan 2014
	Author: Jason Poh
*/

#include "ros/ros.h"
#include "bbauv_msgs/TimeElapsed.h"

bool doYaw(int val)
{

}

bool doRoll(int val)
{

}

bool doPitch(int val)
{

}

bool goToDepth(int metres)
{

}

bool moveLeft(int metres)
{

}

bool moveRight(int metres)
{

}

bool rotateRight(int degrees)
{

}

bool rotateLeft(int degrees)
{

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
