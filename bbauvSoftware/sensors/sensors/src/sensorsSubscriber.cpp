#include <ros/ros.h>
#include <bbauv_msgs/env_data.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/sensors_data.h>
using namespace std;

float yaw,pitch,roll,temperature,Ax,Ay,Az;
float Temp0,Temp1,Temp2,Depth,WaterDetA,WaterDetB,WaterDetC;

void compassCallback(const bbauv_msgs::compass_data::ConstPtr& compassMsg) {
	yaw = compassMsg->yaw;
	pitch = compassMsg->pitch;
	roll = compassMsg->roll;
	temperature = compassMsg->temperature;
	Ax = compassMsg->Ax;
	Ay = compassMsg->Ay;
	Az = compassMsg->Az;	
	ROS_INFO("yaw        : %f\n",yaw);
	ROS_INFO("pitch      : %f\n",pitch);
	ROS_INFO("roll       : %f\n",roll);
	ROS_INFO("temperature: %f\n",temperature);
	ROS_INFO("Ax         : %f\n",Ax);
	ROS_INFO("Ay         : %f\n",Ay);
	ROS_INFO("Az         : %f\n",Az);
}

void envCallBack(const bbauv_msgs::env_data::ConstPtr& envMsg) {
	Temp0 = envMsg->Temp0;
	Temp1 = envMsg->Temp1;
	Temp2 = envMsg->Temp2;
	Depth = envMsg->Depth;
	WaterDetA = envMsg->WaterDetA;
	WaterDetB = envMsg->WaterDetB;
	WaterDetC = envMsg->WaterDetC;
	ROS_INFO("Temp0      : %f\n",Temp0);
	ROS_INFO("Temp1      : %f\n",Temp1);
	ROS_INFO("Temp2      : %f\n",Temp2);
	ROS_INFO("Depth      : %f\n",Depth);
	ROS_INFO("Water Det A: %f\n",WaterDetA);
	ROS_INFO("Water Det B: %f\n",WaterDetB);
	ROS_INFO("Water Det C: %f\n",WaterDetC);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sensorsSubscriber");
	ros::NodeHandle nh;

	ros::Subscriber compassSub = nh.subscribe("os5000_data", 1000, compassCallback);
	ros::Subscriber envSub = nh.subscribe("env_data", 1000, envCallBack);
	ros::Publisher pub = nh.advertise<bbauv_msgs::sensors_data>("sensors_logic",1000);
	bbauv_msgs::sensors_data pubdata;
	ros::Rate loop_rate(10);

	while (nh.ok()) {
		pubdata.yaw = yaw;
		pubdata.pitch = pitch;
		pubdata.roll = roll;
		pubdata.temperature = temperature;
		pubdata.Ax = Ax;
		pubdata.Ay = Ay;
		pubdata.Az = Az;
		pubdata.Temp0 = Temp0;
		pubdata.Temp1 = Temp1;
		pubdata.Temp2 = Temp2;
		pubdata.Depth = Depth;
		pubdata.WaterDetA = WaterDetA;
		pubdata.WaterDetB = WaterDetB;
		pubdata.WaterDetC = WaterDetC;
		pub.publish(pubdata);
		
		ros::spin();
		loop_rate.sleep();
	}
	return 0;
}
