#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
using namespace std;

const double sqrt2 = 1.4142;

int x,y,z,yaw;

void monitorCallBack(const bbauv_msgs::manual_control::ConstPtr& msg) {
	x = msg->x;
	y = msg->y;
    	z = msg->z;
	yaw = msg->yaw;
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"publisher");
	ros::NodeHandle nh;
	bbauv::thruster thrusterMsg;
	ros::Publisher pub = nh.advertise<bbauv_msgs/thruster>("controller_actuator",1000);
	ros::Subcriber sub = nh.subscribe("monitor_controller",1000,monitorCallBack);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		thrusterMsg.speed1 = 3200*z;
		thrusterMsg.speed4 = 3200*z;
		thrusterMsg.speed2 = 3200*(y + yaw)/(-2);
		thrusterMsg.speed3 = 3200*((yaw - y)/sqrt2 - x) / 2;
		thrusterMsg.speed5 = 3200*(-(yaw - y)/sqrt2 - x) / 2;
		pub.publish(thrusterMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
