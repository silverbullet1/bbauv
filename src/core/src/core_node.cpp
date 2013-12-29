#include "ros/ros.h"
#include "tasks/TaskDescriptor.cpp"
#include "msgs/TaskStatus.h"
#include "std_msgs/String.h"
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
string tasks_descriptor_path = "MissionPlans/" + tasks_descriptor_filename;
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

bool taskFeedback(msgs::TaskStatus::Request  &req,
         			msgs::TaskStatus::Response &res)
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
	    			case 1://Fallback Task
	    				task_descriptor.fallback_task = string(token);
	    				break;
	    			case 2://Timeout
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

