#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "core_node");
	ros::Rate rate(1000); //1000 hz
	while(ros::ok())
	{

		r.sleep();
	}
	return 0;
}