#include "ros/ros.h"
#include "tasks/TaskDescriptor.cpp"
#include "bbauv_msgs/TaskStatus.h"
#include "std_msgs/String.h"
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
using namespace std;

/*
	Declarations
*/
ifstream tasks_descriptor_file;
string line;
string tasks_descriptor_filename = "sauvc.txt";
string tasks_descriptor_path = "";
string running_mode = "AUTO";
char * token;
vector<TaskDescriptor> tasks;
string current_general_task = "";
int current_task_id = 0;
std_msgs::String msg;
ros::Time current_task_start_time;

void setCurrentTaskTime()
{
	current_task_start_time = ros::Time::now();
}

void updateCurrentTask()
{
	if(current_task_id>-1)
	{
		current_general_task = tasks[current_task_id].task;
	}
	setCurrentTaskTime();
}

void setFallbackTask()
{
	string fallbackTask = tasks[current_task_id].fallback_task;
	ROS_INFO("Fallback task is %s", fallbackTask.c_str());
	for(int i=0;i<tasks.size();i++)
	{
		string targetTask = tasks[i].task;
		if(fallbackTask.compare(targetTask) == 0)
		{
			current_task_id = i;
			ROS_INFO("Fallback task is found! %s", fallbackTask.c_str());
			return;
		}
	}
	current_task_id = -1;
	current_general_task = "IDLE";
}

bool taskFeedback(bbauv_msgs::TaskStatus::Request  &req,
         			bbauv_msgs::TaskStatus::Response &res)
{
	if(req.isCompleted)
	{
		current_task_id++;
	}
	else
	{
		setFallbackTask();
	}
	updateCurrentTask();
	res.isAcknowledged = true;
	return true;
}

int main(int argc, char **argv)
{
	/*
		Node Initialization
	*/
	ros::init(argc, argv, "core_node");
	ros::NodeHandle node;
	ros::Rate rate(50); //in hz
	ros::ServiceServer taskFeedbackService = node.advertiseService("core_node/task_feedback", taskFeedback);
	ros::Publisher general_task_publisher = node.advertise<std_msgs::String>("core_node/current_task", 10);

	/*
		Get params from ROS Param Server
	*/
	if (node.hasParam("/core/mission_plan"))
	{
		if(node.getParam("/core/mission_plan", tasks_descriptor_filename))
		{
			ROS_INFO("Mission Plan: %s", tasks_descriptor_filename.c_str());
		}
	}
	if (node.hasParam("/core/tasks_descriptor_path"))
	{
		if(node.getParam("/core/tasks_descriptor_path", tasks_descriptor_path))
		{
			tasks_descriptor_path = tasks_descriptor_path + "/" + tasks_descriptor_filename;
			ROS_INFO("Mission Plan Full Path: %s", tasks_descriptor_path.c_str());
		}
	}
	if (node.hasParam("/core/running_mode"))
	{
		if(node.getParam("/core/running_mode", running_mode))
		{
			ROS_INFO("Running Mode: %s", running_mode.c_str());
		}
	}

	/*
		Read our mission plan
	*/
	tasks_descriptor_file.open(tasks_descriptor_path.c_str());
	if(tasks_descriptor_file.is_open())
	{
		ROS_INFO("Reading tasks descriptor file: %s", tasks_descriptor_path.c_str());
		while ( getline(tasks_descriptor_file, line) )
	    {
	    	vector<string> splitVec;
	    	split( splitVec, line, boost::algorithm::is_any_of(", "), boost::token_compress_on );
	    	TaskDescriptor task_descriptor;
	    	task_descriptor.task = splitVec[0];
	    	task_descriptor.fallback_task = splitVec[1];
	    	task_descriptor.timeout = atoi(splitVec[2].c_str());
	    	tasks.push_back(task_descriptor);
	    }
	}
	tasks_descriptor_file.close();
	/*
		Intialize Current Task
	*/
	if(tasks.size()>0)
	{
		current_general_task = tasks[0].task;
		setCurrentTaskTime();
		
	}
		
	/*
		Main Loop
	*/
	while(ros::ok())
	{
		msg.data = current_general_task;
		general_task_publisher.publish(msg);
		int currentTaskTimeElapsed = (ros::Time::now() - current_task_start_time).toSec();
		if(currentTaskTimeElapsed>=tasks[current_task_id].timeout && current_general_task.compare("IDLE") != 0)
		{
			ROS_INFO("TimedOut!");
			setFallbackTask();
			updateCurrentTask();
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

