#include "ros/ros.h"
#include "tasks/TaskDescriptor.cpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
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
	char * token;
	vector<TaskDescriptor> tasks;
	/*
		Read our mission plan
	*/
	ROS_INFO("Reading tasks descriptor file: %s", tasks_descriptor_path.c_str());
	tasks_descriptor_file.open(tasks_descriptor_path.c_str());
	if(tasks_descriptor_file.is_open())
	{
		while ( getline(tasks_descriptor_file, line) )
	    {
	    	ROS_INFO("%s", line.c_str());
	    	TaskDescriptor task_descriptor;
	    	char * cstr = new char [line.length()+1];
	    	strcpy (cstr, line.c_str());

	    	token = strtok (cstr, " ,");
	    	int token_i = 0;
	    	while(token != NULL)
	    	{
	    		//ROS_INFO("%s", token);
	    		switch(token_i)
	    		{
	    			case 0://Task
	    				task_descriptor.task = string(token);
	    				break;
	    			case 1://Next Task
	    				task_descriptor.next_task = string(token);
	    				break;
	    			case 2://Fallback Task
	    				task_descriptor.fallback_task = string(token);
	    				break;
	    			case 3://Timeout
	    				task_descriptor.timeout = atoi(token);
	    				tasks.push_back(task_descriptor);
	    				break;
	    		}
	    		token = strtok (NULL, " ,");
	    		token_i++;
	    	}
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