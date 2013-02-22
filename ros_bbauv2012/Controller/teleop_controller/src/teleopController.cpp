#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <bbauv_msgs/thruster.h>
#include <dynamic_reconfigure/server.h>
#include <teleop_controller/thrusterRatiosConfig.h>
#include <sensor_msgs/Joy.h>

using namespace std;

const float sqrt2 = 1.4142;
const int mapRatio = 2560;
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
bool reset = false;
float x,y,z,yaw;

void joyTranslate(const sensor_msgs::Joy::ConstPtr& joy)
{

  x=joy->axes[1];
  y=joy->axes[0];
  z=joy->axes[3];
  yaw=-joy->axes[2]; //need to check the turn direction
// Notice: in controllerCode. (+) direction is clock-wise
  yaw_mode=joy->buttons[1];
  xy_mode=!yaw_mode;
 
  //scaling for yaw:
  if(yaw>0) yaw=yaw*1.0/0.24;
  if(yaw>1) yaw=1;
}

void callback(teleop_controller::thrusterRatiosConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %s %s %s", 
			config.motor_test_mode?"Motor_Testing":"Teleop_Mode",
			config.z_mode?"Depth_Control":"No_Depth_Control",
			config.reset?"Stopped":"Normal");
	test_mode = config.motor_test_mode;

	ratio1 = config.thruster1 * mapRatio;
	ratio2 = config.thruster2 * mapRatio;
	ratio3 = config.thruster3 * mapRatio;
	ratio4 = config.thruster4 * mapRatio;
	ratio5 = config.thruster5 * mapRatio;
	ratio6 = config.thruster6 * mapRatio;
	z_mode = config.z_mode;
	reset = config.reset;	
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

	ros::Publisher pub = nh.advertise<bbauv_msgs::thruster>("teleop_controller",20);
	ros::Subscriber sub = nh.subscribe("joy",20,joyTranslate);
	ros::Rate loop_rate(16);
	float absx,absy,absmax;
	while (ros::ok()) {
		if(reset)
		{
			thrusterMsg.speed1 = 0;
			thrusterMsg.speed2 = 0;
			thrusterMsg.speed3 = 0;
			thrusterMsg.speed4 = 0;
			thrusterMsg.speed5 = 0;
			thrusterMsg.speed6 = 0;
		}
		else if (test_mode == false) {
			if (z_mode == true) {
				thrusterMsg.speed5 = mapRatio*z;
				thrusterMsg.speed6 = mapRatio*z;
			}
			if (yaw_mode == true) {
				ROS_DEBUG("yaw axis");
				thrusterMsg.speed1 = mapRatio*yaw*0.5;
				thrusterMsg.speed2 = -mapRatio*yaw*0.5;
				thrusterMsg.speed3 = mapRatio*yaw*0.5;
				thrusterMsg.speed4 = -mapRatio*yaw*0.5;
			}
			if (xy_mode == true) {
				if (absolute(x) >= absolute(y)) {
					ROS_DEBUG("x axis");
					thrusterMsg.speed1 = -mapRatio*x;
					thrusterMsg.speed2 = -mapRatio*x;
					thrusterMsg.speed3 = mapRatio*x;
					thrusterMsg.speed4 = mapRatio*x;
				}
				else {
					ROS_DEBUG("y axis");
					thrusterMsg.speed1 = mapRatio*y;
					thrusterMsg.speed2 = -mapRatio*y;
					thrusterMsg.speed3 = -mapRatio*y;
					thrusterMsg.speed4 = mapRatio*y;
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
