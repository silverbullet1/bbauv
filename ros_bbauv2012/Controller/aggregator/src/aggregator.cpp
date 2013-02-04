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

using namespace std;

//aggregate sensor feedback
void update_setpoint(const geometry_msgs::Twist sp);
void collect_depth(const bbauv_msgs::env_data& msg);
void collect_heading(const bbauv_msgs::compass_data& msg);
  //for fwd, bwd, sidemove. Rmb to change msg type & edit the func below
void collect_velocity(const nav_msgs::Odometry::ConstPtr& msg);

void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level); 

bbauv_msgs::controller_input ctrl;
bbauv_msgs::controller_onoff mode;
bbauv_msgs::controller_translational_constants trans_const;
bbauv_msgs::controller_rotational_constants rot_const;

ros::Publisher controller_input_pub;
ros::Publisher controller_mode_pub;
ros::Publisher controller_trans_const_pub;
ros::Publisher controller_rot_const_pub;

ros::Subscriber cmd_vel_sub; //used to be controller_setpoint_sub
ros::Subscriber depth_sub; 
ros::Subscriber compass_sub;
//name can be updated
ros::Subscriber velocity_sub; 

double yaw_radians;

int main(int argc,char** argv) {
  ros::init(argc,argv,"aggregator");
  ros::NodeHandle nh;
  
  //subscribers declaration
  cmd_vel_sub = nh.subscribe("cmd_vel",20,update_setpoint,ros::TransportHints().tcpNoDelay());
  depth_sub = nh.subscribe("env_data",20,collect_depth,ros::TransportHints().tcpNoDelay());
  compass_sub = nh.subscribe("os5000_data",20,collect_heading,ros::TransportHints().tcpNoDelay());
    //topic name, msg type need to be updated
  velocity_sub = nh.subscribe("odom",20,collect_velocity,ros::TransportHints().tcpNoDelay());
  
  //publishers declaration
  controller_input_pub = nh.advertise<bbauv_msgs::controller_input>("controller_input",20);
  controller_mode_pub = nh.advertise<bbauv_msgs::controller_onoff>("controller_mode",20);
  controller_trans_const_pub = nh.advertise<bbauv_msgs::controller_translational_constants>("translational_constants",20);
  controller_rot_const_pub = nh.advertise<bbauv_msgs::controller_rotational_constants>("rotational_constants",20);


  //dynamic reconfigure
  dynamic_reconfigure::Server<aggregator::controller_paramConfig> server;
  dynamic_reconfigure::Server<aggregator::controller_paramConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  //finish setup and declaration, go to loop
  ros::Rate loop_rate(5);
  while (ros::ok()) {

/*
    if((ctrl.forward_setpoint== 0 || ctrl.forward_setpoint ==0.1) && ctrl.heading_setpoint!= 0)
    {
	mode.forward_PID=false;
        mode.heading_PID=true;
  	mode.sidemove_PID=true;
    }
    else if (ctrl.forward_setpoint!= 0 && ctrl.heading_setpoint== 0)
    {
    	mode.forward_PID=true;       
        mode.heading_PID=false;
	mode.sidemove_PID=true;
    }
    else
    {
      if(mode.forward_PID)
      {
	  mode.forward_PID=false;
          mode.heading_PID=true;
	  mode.sidemove_PID=true;
      }
      else
      {
	  mode.forward_PID=true;       
          mode.heading_PID=false;
	  mode.sidemove_PID=true;
      }
    }
*/

    controller_input_pub.publish(ctrl);
    controller_mode_pub.publish(mode);
    controller_trans_const_pub.publish(trans_const);
    controller_rot_const_pub.publish(rot_const);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void update_setpoint(const geometry_msgs::Twist sp)
{
  //ctrl.depth_setpoint=sp.depth_setpoint;
  ctrl.heading_setpoint= sp.angular.z;
  ctrl.forward_setpoint=sp.linear.x;
  ctrl.sidemove_setpoint=sp.linear.y;
  //ctrl.backward_setpoint=sp.backward_setpoint;
  //ctrl.sidemove_setpoint=sp.sidemove_setpoint;
}

void collect_depth(const bbauv_msgs::env_data& msg)
{
  ctrl.depth_input = msg.Depth;  
}

void collect_heading(const bbauv_msgs::compass_data& msg)
{
  
}


//this is what we are doing when we receive the velocity data
//the goal is to find out the input to send to PID
//ctrl.forward_input, ctrl.backward_input, ctrl.sidemove
void collect_velocity(const nav_msgs::Odometry::ConstPtr& msg)
{
  ctrl.forward_input = msg->twist.twist.linear.x;

  ctrl.heading_input = msg->twist.twist.angular.z;
  
  ctrl.sidemove_input = msg->twist.twist.linear.y;
}


void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level) 
{
  trans_const.depth_kp=config.depth_kp;	
  trans_const.depth_ki=config.depth_ki;	
  trans_const.depth_kd=config.depth_kd;
  //ctrl.depth_setpoint=config.depth_setpoint;
	
  trans_const.forward_kp=config.forward_kp;
  trans_const.forward_ki=config.forward_ki;
  trans_const.forward_kd=config.forward_kd;
  //ctrl.forward_setpoint=config.forward_setpoint;
  //ctrl.forward_input=config.forward_input;

  trans_const.backward_kp=config.backward_kp;
  trans_const.backward_ki=config.backward_ki;
  trans_const.backward_kd=config.backward_kd;
  //ctrl.backward_setpoint=config.backward_setpoint;

  trans_const.sidemove_kp=config.sidemove_kp;
  trans_const.sidemove_ki=config.sidemove_ki;
  trans_const.sidemove_kd=config.sidemove_kd;
  //ctrl.sidemove_setpoint=config.sidemove_setpoint;

  rot_const.heading_kp=config.heading_kp;
  rot_const.heading_ki=config.heading_ki;
  rot_const.heading_kd=config.heading_kd;
  //ctrl.heading_setpoint=config.heading_setpoint;

  param.ratio_t1=config.ratio_t1;	
  param.ratio_t2=config.ratio_t2;	
  param.ratio_t3=config.ratio_t3;	
  param.ratio_t4=config.ratio_t4;	
  param.ratio_t5=config.ratio_t5;	
  param.ratio_t6=config.ratio_t6;	
*/

  mode.depth_PID=config.depth_PID;
  mode.heading_PID=config.heading_PID;
  mode.forward_PID=config.forward_PID;
  mode.backward_PID=config.backward_PID;
  mode.sidemove_PID=config.sidemove_PID;
  mode.teleop=config.teleop;

  //mode.reset=config.reset;

}

