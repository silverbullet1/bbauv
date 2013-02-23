#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <bbauv_msgs/controller_input.h>
#include <bbauv_msgs/controller_setpoint.h>
#include <bbauv_msgs/controller_onoff.h>
#include <bbauv_msgs/controller_translational_constants.h>
#include <bbauv_msgs/controller_rotational_constants.h>
#include <bbauv_msgs/env_data.h>
#include <bbauv_msgs/compass_data.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <aggregator/controller_paramConfig.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <bbauv_msgs/thruster.h>

using namespace std;

/* Global Variable declaration */

double depthAtSurface;
const float sqrt2 = 1.4142;
const int mapRatio = 2560;
bool yaw_mode = false;
bool xy_mode = false;
float x,y,z,yaw;

/* Function prototypes */

void update_move_base_setpoint(const geometry_msgs::Twist sp);
void update_tracker_setpoint (const bbauv_msgs::controller_input track);

void collect_depth(const bbauv_msgs::env_data& msg);
void collect_heading(const bbauv_msgs::compass_data& msg);
void collect_velocity(const nav_msgs::Odometry::ConstPtr& msg);
void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level); 
void joyTranslate(const sensor_msgs::Joy::ConstPtr& joy);


/* ROS Initialization */

//bbauv_msgs::controller_input cmd_vel;
bbauv_msgs::controller_input ctrl;
bbauv_msgs::controller_onoff mode;
bbauv_msgs::controller_translational_constants trans_const;
bbauv_msgs::controller_rotational_constants rot_const;
bbauv_msgs::thruster thrusterMsg;

ros::Publisher controller_input_pub;
ros::Publisher controller_mode_pub;
ros::Publisher controller_trans_const_pub;
ros::Publisher controller_rot_const_pub;
ros::Publisher pub;

ros::Subscriber tracker_sub;
ros::Subscriber cmd_vel_sub; //for move_base
ros::Subscriber depth_sub; 
ros::Subscriber compass_sub;
ros::Subscriber velocity_sub;
ros::Subscriber sub;


float absolute(float input) {

	if (input < 0) return (-input);
	else return input;
}

/************ Main Loop *****************/

int main(int argc,char** argv) {

  ros::init(argc,argv,"aggregator");
  ros::NodeHandle nh;
 
  //subscribers declaration
  cmd_vel_sub = nh.subscribe("cmd_vel",20,update_move_base_setpoint,ros::TransportHints().tcpNoDelay());
  tracker_sub = nh.subscribe("/line_follower",20,update_tracker_setpoint,ros::TransportHints().tcpNoDelay());
  depth_sub = nh.subscribe("env_data",20,collect_depth,ros::TransportHints().tcpNoDelay());
  compass_sub = nh.subscribe("os5000_data",20,collect_heading,ros::TransportHints().tcpNoDelay());
  velocity_sub = nh.subscribe("odom",20,collect_velocity,ros::TransportHints().tcpNoDelay());
  sub = nh.subscribe("joy",20,joyTranslate);
  
  //publishers declaration
  controller_input_pub = nh.advertise<bbauv_msgs::controller_input>("controller_input",20);
  controller_mode_pub = nh.advertise<bbauv_msgs::controller_onoff>("controller_mode",20);
  controller_trans_const_pub = nh.advertise<bbauv_msgs::controller_translational_constants>("translational_constants",20);
  controller_rot_const_pub = nh.advertise<bbauv_msgs::controller_rotational_constants>("rotational_constants",20);
	pub = nh.advertise<bbauv_msgs::thruster>("teleop_controller",20);

  //dynamic reconfigure
  dynamic_reconfigure::Server<aggregator::controller_paramConfig> server;
  dynamic_reconfigure::Server<aggregator::controller_paramConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  //finish setup and declaration, go to loop
  ros::Rate loop_rate(16);
  while (ros::ok()) {

    //get Parameters from Param Server
        //Due to the use of an absolute pressure sensor, we need to subtract the pressure at atm bef we enter the water in order to obtain accurate depths
    nh.getParam("/aggregator/depthAtSurface",depthAtSurface); 
    
    if (yaw_mode == true) {
	    ROS_DEBUG("yaw axis");
	    thrusterMsg.speed1 = mapRatio*yaw*0.5;
	    thrusterMsg.speed2 = -mapRatio*yaw*0.5;
	    thrusterMsg.speed3 = mapRatio*yaw*0.5;
	    thrusterMsg.speed4 = -mapRatio*yaw*0.5;
	    thrusterMsg.speed5 = mapRatio*z;
	    thrusterMsg.speed6 = mapRatio*z;
    }
    if (xy_mode == true) {
	    if (absolute(x) >= absolute(y)) {
		    ROS_DEBUG("x axis");
		    thrusterMsg.speed1 = -mapRatio*x;
		    thrusterMsg.speed2 = -mapRatio*x;
		    thrusterMsg.speed3 = mapRatio*x;
		    thrusterMsg.speed4 = mapRatio*x;
	        thrusterMsg.speed5 = mapRatio*z;
	        thrusterMsg.speed6 = mapRatio*z;
	    }
    
	    else {
		    ROS_DEBUG("y axis");
		    thrusterMsg.speed1 = mapRatio*y;
		    thrusterMsg.speed2 = -mapRatio*y;
		    thrusterMsg.speed3 = -mapRatio*y;
		    thrusterMsg.speed4 = mapRatio*y;
	        thrusterMsg.speed5 = mapRatio*z;
	        thrusterMsg.speed6 = mapRatio*z;
	    }
    }


    controller_input_pub.publish(ctrl);   
    pub.publish(thrusterMsg);           
    controller_mode_pub.publish(mode);
    controller_trans_const_pub.publish(trans_const);
    controller_rot_const_pub.publish(rot_const);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/* ROS Callback functions */

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

void update_move_base_setpoint(const geometry_msgs::Twist sp)
{
/*
  cmd_vel.depth_setpoint=sp.depth_setpoint;
  cmd_vel.heading_setpoint= sp.heading_setpoint;
  cmd_vel.forward_setpoint=sp.forward_setpoint;
  cmd_vel.sidemove_setpoint=sp.sidemove_setpoint;
*/
}

void update_tracker_setpoint(const bbauv_msgs::controller_input track)
{
  ctrl.depth_setpoint=track.depth_setpoint;
  ctrl.heading_setpoint=track.heading_setpoint;
  ctrl.forward_setpoint=track.forward_setpoint;
  ctrl.sidemove_setpoint=track.sidemove_setpoint;
}

void collect_depth(const bbauv_msgs::env_data& msg)
{
  ctrl.depth_input = msg.Depth;
  ctrl.depth_input -= depthAtSurface;
}

void collect_heading(const bbauv_msgs::compass_data& msg)
{
  ctrl.heading_input = msg.yaw;
}


//this is what we are doing when we receive the velocity data
//the goal is to find out the input to send to PID
//ctrl.forward_input, ctrl.backward_input, ctrl.sidemove
void collect_velocity(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ctrl.forward_input = msg->twist.twist.linear.x;

  //ctrl.heading_input = msg->twist.twist.angular.z;
  
  //ctrl.sidemove_input = msg->twist.twist.linear.y;
}


void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level) 
{
  trans_const.depth_kp=config.depth_kp;	
  trans_const.depth_ki=config.depth_ki;	
  trans_const.depth_kd=config.depth_kd;
  //ctrl.depth_setpoint=config.depth_setpoint;

  rot_const.heading_kp=config.heading_kp;
  rot_const.heading_ki=config.heading_ki;
  rot_const.heading_kd=config.heading_kd;
  //ctrl.heading_setpoint=config.heading_setpoint;
	
  trans_const.forward_kp=config.forward_kp;
  trans_const.forward_ki=config.forward_ki;
  trans_const.forward_kd=config.forward_kd;
  //ctrl.forward_setpoint=config.forward_setpoint;
  //ctrl.forward_input=config.forward_input;

  trans_const.sidemove_kp=config.sidemove_kp;
  trans_const.sidemove_ki=config.sidemove_ki;
  trans_const.sidemove_kd=config.sidemove_kd;
  //ctrl.sidemove_setpoint=config.sidemove_setpoint;

  mode.depth_PID=config.depth_PID;
  mode.heading_PID=config.heading_PID;
  mode.forward_PID=config.forward_PID;
  mode.sidemove_PID=config.sidemove_PID;  
  mode.topside=config.topside;
  mode.teleop=config.teleop;

}

