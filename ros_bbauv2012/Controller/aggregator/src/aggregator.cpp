#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <bbauv_msgs/controller_input.h>
#include <bbauv_msgs/controller_setpoint.h>
#include <bbauv_msgs/controller_param.h>
#include <bbauv_msgs/env_data.h>
#include <bbauv_msgs/compass_data.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <aggregator/controller_paramConfig.h>

using namespace std;

void update_setpoint(const bbauv_msgs::controller_setpoint sp);
void collect_depth(const bbauv_msgs::env_data& msg);
void collect_heading(const bbauv_msgs::compass_data& msg);

void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level); 

bbauv_msgs::controller_input ctrl;
bbauv_msgs::controller_param param;

ros::Publisher controller_param_pub;
ros::Publisher controller_input_pub;
ros::Subscriber depth_sub; 
ros::Subscriber compass_sub; 
ros::Subscriber controller_setpoint_sub; 

int main(int argc,char** argv) {
  ros::init(argc,argv,"aggregator");
  ros::NodeHandle nh;
  //subscribers declaration
  depth_sub = nh.subscribe("env_data",20,collect_depth,ros::TransportHints().tcpNoDelay());
  compass_sub = nh.subscribe("os5000_data",20,collect_heading,ros::TransportHints().tcpNoDelay());
  controller_setpoint_sub = nh.subscribe("controller_setpoint",20,update_setpoint,ros::TransportHints().tcpNoDelay());
  
  //publishers declaration
  controller_input_pub = nh.advertise<bbauv_msgs::controller_input>("controller_input",20);
  controller_param_pub = nh.advertise<bbauv_msgs::controller_param>("controller_config",20);

  //dynamic reconfigure
  dynamic_reconfigure::Server<aggregator::controller_paramConfig> server;
  dynamic_reconfigure::Server<aggregator::controller_paramConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  //finish setup and declaration, go to loop
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    controller_input_pub.publish(ctrl);
    controller_param_pub.publish(param);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void update_setpoint(const bbauv_msgs::controller_setpoint sp)
{
  ctrl.depth_setpoint=sp.depth_setpoint;
  ctrl.heading_setpoint=sp.heading_setpoint;
}

void collect_depth(const bbauv_msgs::env_data& msg)
{
  ctrl.depth_input = msg.Depth;  
}
void collect_heading(const bbauv_msgs::compass_data& msg)
{
  ctrl.heading_input=msg.yaw;
}

void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level) 
{
  param.depth_kp=config.depth_kp;	
  param.depth_ki=config.depth_ki;	
  param.depth_kd=config.depth_kd;
	
  param.heading_kp=config.heading_kp;
  param.heading_ki=config.heading_ki;
  param.heading_kd=config.heading_kd;
		
  param.ratio_t1=config.ratio_t1;	
  param.ratio_t2=config.ratio_t2;	
  param.ratio_t3=config.ratio_t3;	
  param.ratio_t4=config.ratio_t4;	
  param.ratio_t5=config.ratio_t5;	
  param.ratio_t6=config.ratio_t6;	
  param.depth_PID=config.depth_PID;
  param.heading_PID=config.heading_PID;
  param.reset=config.reset;
}

