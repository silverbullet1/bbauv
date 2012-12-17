#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
using namespace std;

const float sqrt2 = 1.4142;
const int mapRatio = 2500;
float x,y,z,yaw;

void monitorCallBack(const bbauv_msgs::manual_control::ConstPtr& msg) {
	x = msg->x;
	y = msg->y;
	z = msg->z;
	yaw = msg->yaw;
}

float abs(float input) {
	if (input < 0) return (-input);
	else return input;
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"thrusterPublisher");
	ros::NodeHandle nh;
	bbauv_msgs::thruster thrusterMsg;
	ros::Publisher pub = nh.advertise<bbauv_msgs::thruster>("motor_controller",1000);
	ros::Subscriber sub = nh.subscribe("monitor_controller",1000,monitorCallBack);
	ros::Rate loop_rate(10);
	int absx,absy,absyaw,absmax;
	while (ros::ok()) {
		thrusterMsg.speed1 = mapRatio*z;
		thrusterMsg.speed4 = mapRatio*z;
		absx = abs(x);
		absy = abs(y);
		absyaw = abs(yaw);
		//find the largest absoluted number
		absmax = absx;
		if (absy > absmax) absmax = absy;
		if (absyaw > absmax) absmax = absyaw;
		
		if (absmax = absx) {
			thrusterMsg.speed2 = 0;
			thrusterMsg.speed3 = mapRatio*x;
			thrusterMsg.speed5 = mapRatio*x;
		}
		else if (absmax = absy) {
			thrusterMsg.speed2 = mapRatio*y;
			thrusterMsg.speed3 = mapRatio*y/sqrt2;
			thrusterMsg.speed5 = -mapRatio*y/sqrt2;
		}
		else {
			thrusterMsg.speed2 = mapRatio*yaw;
			thrusterMsg.speed3 = -mapRatio*yaw/sqrt2;
			thrusterMsg.speed5 = mapRatio*yaw/sqrt2;
		}
		pub.publish(thrusterMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
