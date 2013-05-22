#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;

void call_back(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv) {
	ros::init(argc, argv, "Q2E");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("WH_DVL_data", 1000, call_back);
	ros::spin();
	return 0;
}

void call_back(const nav_msgs::Odometry::ConstPtr& msg) {
	double q0 = (msg->pose).pose.orientation.w;
	double q1 = (msg->pose).pose.orientation.x;
	double q2 = (msg->pose).pose.orientation.y;
	double q3 = (msg->pose).pose.orientation.z;
	double r,p,y;

	r = atan2(2 * (q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
	p = asin(2 * (q0*q2 - q3*q1));
	y = atan2(2 * (q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));

	r = r * 180.0 / M_PI;
	p = p * 180.0 / M_PI;
	y = y * 180.0 / M_PI;

	ROS_INFO("Q2E Call Back - Roll, Pitch, Yaw");
	printf("%06.2lf -- %06.2lf -- %06.2lf\n", r, p ,y);
}
