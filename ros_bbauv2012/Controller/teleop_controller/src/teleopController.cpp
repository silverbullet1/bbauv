#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
#include <dynamic_reconfigure/server.h>
#include <teleop_controller/thrusterRatiosConfig.h>
using namespace std;

const float sqrt2 = 1.4142;
const int mapRatio = 2500;
int ratio1 = mapRatio;
int ratio2 = mapRatio;
int ratio3 = mapRatio;
int ratio4 = mapRatio;
int ratio5 = mapRatio;
int ratio6 = mapRatio;
bool test_mode = false;
bool z_mode = false;
bool yaw_mode = false;
bool xy_mode = false;
float x,y,z,yaw;

void monitorCallBack(const bbauv_msgs::manual_control::ConstPtr& msg) {
	x = msg->x;
	y = msg->y;
	z = msg->z;
	yaw = msg->yaw;
}

void callback(teleop_controller::thrusterRatiosConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %s %s %s", 
			config.thruster1, config.thruster2, 
			config.thruster3, config.thruster4,
			config.thruster5, config.thruster6,
			config.motor_test_mode?"True":"False",
			config.z_mode?"True":"False",
			config.xy_mode?"True":"False");
	ratio1 = config.thruster1 * mapRatio;
	ratio2 = config.thruster2 * mapRatio;
	ratio3 = config.thruster3 * mapRatio;
	ratio4 = config.thruster4 * mapRatio;
	ratio5 = config.thruster5 * mapRatio;
	ratio6 = config.thruster6 * mapRatio;
	test_mode = config.motor_test_mode;
	z_mode = config.z_mode;
	yaw_mode = config.yaw_mode;
	xy_mode = config.xy_mode;
}

float absolute(float input) {

	if (input < 0) return (-input);
	else return input;
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"teleopController");
	ros::NodeHandle nh;
	bbauv_msgs::thruster thrusterMsg;

	dynamic_reconfigure::Server<teleop_controller::thrusterRatiosConfig> server;
	dynamic_reconfigure::Server<teleop_controller::thrusterRatiosConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::Publisher pub = nh.advertise<bbauv_msgs::thruster>("motor_controller",1000);
	ros::Subscriber sub = nh.subscribe("monitor_controller",1000,monitorCallBack);
	ros::Rate loop_rate(10);
	float absx,absy,absmax;
	while (ros::ok()) {
		if (test_mode == false) {
			if (z_mode == true) {
				thrusterMsg.speed5 = ratio5*z;
				thrusterMsg.speed6 = ratio6*z;
			}
			if (yaw_mode == true) {
				ROS_DEBUG("yaw axis");
				thrusterMsg.speed1 = -ratio1*yaw;
				thrusterMsg.speed2 = ratio2*yaw;
				thrusterMsg.speed3 = -ratio3*yaw;
				thrusterMsg.speed4 = ratio4*yaw;
			}
			if (xy_mode == true) {
				if (absolute(x) >= absolute(y)) {
					ROS_DEBUG("x axis");
					thrusterMsg.speed1 = -ratio1*x;
					thrusterMsg.speed2 = -ratio2*x;
					thrusterMsg.speed3 = ratio3*x;
					thrusterMsg.speed4 = ratio4*x;
				}
				else {
					ROS_DEBUG("y axis");
					thrusterMsg.speed1 = ratio1*y;
					thrusterMsg.speed2 = -ratio2*y;
					thrusterMsg.speed3 = -ratio3*y;
					thrusterMsg.speed4 = ratio4*y;
				}
			}
		}
		else {
			thrusterMsg.speed1 = ratio1;
			thrusterMsg.speed2 = ratio2;
			thrusterMsg.speed3 = ratio3;
			thrusterMsg.speed4 = ratio4;
			thrusterMsg.speed5 = ratio5;
			thrusterMsg.speed6 = ratio6;
		}
		ROS_DEBUG("%d %d %d %d %d %d\n",thrusterMsg.speed1,thrusterMsg.speed2,thrusterMsg.speed3,thrusterMsg.speed4,thrusterMsg.speed5,thrusterMsg.speed6);
		pub.publish(thrusterMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
