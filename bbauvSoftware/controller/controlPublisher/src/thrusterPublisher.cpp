#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
#include <dynamic_reconfigure/server.h>
#include <controlPublisher/thrusterRatiosConfig.h>
using namespace std;

const float sqrt2 = 1.4142;
const int mapRatio = 2500;
int ratio1 = mapRatio;
int ratio2 = mapRatio;
int ratio3 = mapRatio;
int ratio4 = mapRatio;
int ratio5 = mapRatio;
bool test_mode = false;
float x,y,z,yaw;

void monitorCallBack(const bbauv_msgs::manual_control::ConstPtr& msg) {
	x = msg->x;
	y = msg->y;
	z = msg->z;
	yaw = msg->yaw;
}

void callback(controlPublisher::thrusterRatiosConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f %f %f %s", 
			config.thruster1, config.thruster2, 
			config.thruster3, config.thruster4,
			config.thruster5, config.motor_test_mode?"True":"False");
	ratio1 = config.thruster1 * mapRatio;
	ratio2 = config.thruster2 * mapRatio;
	ratio3 = config.thruster3 * mapRatio;
	ratio4 = config.thruster4 * mapRatio;
	ratio5 = config.thruster5 * mapRatio;
	test_mode = config.motor_test_mode;
}

float absolute(float input) {

	if (input < 0) return (-input);
	else return input;
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"thrusterPublisher");
	ros::NodeHandle nh;
	bbauv_msgs::thruster thrusterMsg;

	dynamic_reconfigure::Server<controlPublisher::thrusterRatiosConfig> server;
	dynamic_reconfigure::Server<controlPublisher::thrusterRatiosConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::Publisher pub = nh.advertise<bbauv_msgs::thruster>("motor_controller",1000);
	ros::Subscriber sub = nh.subscribe("monitor_controller",1000,monitorCallBack);
	ros::Rate loop_rate(10);
	float absx,absy,absyaw,absmax;
	while (ros::ok()) {
		if (test_mode == false) {
			thrusterMsg.speed1 = ratio1*z;
			thrusterMsg.speed4 = ratio4*z;
			absx = absolute(x);
			absy = absolute(y);
			absyaw = absolute(yaw);
			//find the largest absoluted number
			absmax = absx;
			if (absy > absmax) absmax = absy;
			if (absyaw > absmax) absmax = absyaw;
			ROS_DEBUG("%f %f %f %f\n",absx,absy,absyaw,absmax);

			if (absmax == absx) {
				ROS_DEBUG("absx");
				thrusterMsg.speed2 = 0;
				thrusterMsg.speed3 = ratio3*x;
				thrusterMsg.speed5 = ratio5*x;
			}
			else if (absmax == absy) {
				ROS_DEBUG("absy");
				thrusterMsg.speed2 = ratio2*y;
				thrusterMsg.speed3 = ratio3*y/sqrt2;
				thrusterMsg.speed5 = -ratio5*y/sqrt2;
			}
			else {
				ROS_DEBUG("absyaw");
				thrusterMsg.speed2 = ratio2*yaw;
				thrusterMsg.speed3 = -ratio3*yaw/sqrt2;
				thrusterMsg.speed5 = ratio5*yaw/sqrt2;
			}
		}
		else {
			thrusterMsg.speed1 = ratio1;
			thrusterMsg.speed2 = ratio2;
			thrusterMsg.speed3 = ratio3;
			thrusterMsg.speed4 = ratio4;
			thrusterMsg.speed5 = ratio5;
		}
		ROS_DEBUG("%d %d %d %d %d\n",thrusterMsg.speed1,thrusterMsg.speed2,thrusterMsg.speed3,thrusterMsg.speed4,thrusterMsg.speed5);
		pub.publish(thrusterMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
