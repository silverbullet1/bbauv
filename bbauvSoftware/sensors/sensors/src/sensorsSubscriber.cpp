#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bbauv_msgs/compass_data.h>
using namespace std;

void compassCallback(const bbauv_msgs::compass_data::ConstPtr& compassMsg) {
	ROS_INFO("Heading: %f\n", compassMsg->yaw);
	ROS_INFO("pitch: %f\n", compassMsg->pitch);
	ROS_INFO("roll: %f\n", compassMsg->roll);
	ROS_INFO("temperature: %f\n", compassMsg->temperature);
	ROS_INFO("Ax: %f\n",compassMsg->Ax);
	ROS_INFO("Ay: %f\n",compassMsg->Ay);
	ROS_INFO("Az: %f\n",compassMsg->Az);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sensorsSubscriber");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("os5000_data", 1000, compassCallback);

	ros::spin();
	return 0;
}
