#include "ros/ros.h"
#include "tasks/TaskDescriptor.cpp"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "core_node");
	/*
		Declarations
	*/
	ros::NodeHandle node;
	ros::Rate rate(50); //in hz
	ifstream tasks_descriptor_file;
	string line;
	string tasks_descriptor_filename = "sauvc.txt";
	string tasks_descriptor_path = "MissionPlans/" + tasks_descriptor_filename;
	TaskDescriptor task_descriptor;
	/*
		Read our mission plan
	*/
	ROS_INFO(("Reading tasks descriptor file: " + tasks_descriptor_path).c_str());
	tasks_descriptor_file.open(tasks_descriptor_path.c_str());
	if(tasks_descriptor_file.is_open())
	{
		while ( getline(tasks_descriptor_file, line) )
	    {
	    	ROS_INFO(line.c_str());
	    }
	}
	tasks_descriptor_file.close();

	/*
		Main Loop
	*/
	while(ros::ok())
	{
		rate.sleep();
	}
	return 0;
}