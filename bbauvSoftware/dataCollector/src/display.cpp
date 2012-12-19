#include <ros/ros.h>
#include <bbauv_msgs/env_data.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/sensors_data.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
#include <sstream>
using namespace std;

float yaw,pitch,roll,temperature,Ax,Ay,Az;
float Temp0,Temp1,Temp2,Depth,WaterDetA,WaterDetB,WaterDetC;
float speed1,speed2,speed3,speed4,speed5;
float monitorX,monitorY,monitorZ,monitorYaw;

void compassCallback(const bbauv_msgs::compass_data::ConstPtr& compassMsg) {
	yaw = compassMsg->yaw;
	pitch = compassMsg->pitch;
	roll = compassMsg->roll;
	temperature = compassMsg->temperature;
	Ax = compassMsg->Ax;
	Ay = compassMsg->Ay;
	Az = compassMsg->Az;	
}

void envCallBack(const bbauv_msgs::env_data::ConstPtr& envMsg) {
	Temp0 = envMsg->Temp0;
	Temp1 = envMsg->Temp1;
	Temp2 = envMsg->Temp2;
	Depth = envMsg->Depth;
	WaterDetA = envMsg->WaterDetA;
	WaterDetB = envMsg->WaterDetB;
	WaterDetC = envMsg->WaterDetC;
}

void monitorCallback(const bbauv_msgs::manual_control::ConstPtr& monitorMsg) {
	monitorX = monitorMsg->x;
	monitorY = monitorMsg->y;
	monitorZ = monitorMsg->z;
	monitorYaw = monitorMsg->yaw; 
}

void thrusCallback(const bbauv_msgs::thruster::ConstPtr& thrusMsg) {
	speed1 = thrusMsg->speed1;
	speed2 = thrusMsg->speed2;
	speed3 = thrusMsg->speed3;
	speed4 = thrusMsg->speed4;
	speed5 = thrusMsg->speed5;	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "display");
	ros::NodeHandle nh;

	ros::Subscriber compassSub = nh.subscribe("os5000_data", 1000, compassCallback);
	ros::Subscriber envSub = nh.subscribe("env_data", 1000, envCallBack);
	ros::Subscriber monitorSub = nh.subscribe("monitor_controller",1000,monitorCallback);
	ros::Subscriber thrusSub = nh.subscribe("motor_controller",1000,thrusCallback);
	
	ros::Rate loop_rate(10);
	string output;

	while (ros::ok()) {
		ostringstream out;
		out << "compass data: " << endl;
		out << "yaw pitch roll  : " << yaw <<" - "<< pitch <<" - "<< roll << endl;
		out << "Ax Ay Az        : " << Ax <<" - "<< Ay <<" - "<< Az << endl;
		out << "temperature     : " << temperature << endl;
		out << "env data    : " << endl;
		out << "temperature 012 : " << Temp0 <<" - "<< Temp1 <<" - "<< Temp2 << endl;
		out << "water detect ABC: " <<WaterDetA<<" - "<<WaterDetB<<" - "<<WaterDetC<< endl;
		out << "Depth           : " << Depth << endl;
		out << "monitor: " << endl;
		out << "x y z yaw       : " << monitorX <<" - "<< monitorY <<" - "<< monitorZ <<" - "<< monitorYaw << endl;
		out << "thruster: " << endl;
		out << "speed1-5        : " << speed1 <<" - "<< speed2 <<" - "<< speed3 <<" - "<< speed4 <<" - "<< speed5 << endl;
		
		ROS_INFO((out.str()).c_str()); 
					
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
