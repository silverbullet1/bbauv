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
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseWithCovariance.h>
//#include <geometry_msgs/TwistWithCovariance.h>

using namespace std;

/* Global Variable declaration */

double depthAtSurface;
bool inTopside;
bool inStateMachine;
bool inNavigation;
bool inVisionTracking;

/* Function prototypes */

void update_move_base_setpoint(const geometry_msgs::Twist sp);
void update_depth_base_setpoint(const bbauv_msgs::controller_input depth);
void update_vision_tracker_setpoint (const bbauv_msgs::controller_input track);

void collect_depth(const bbauv_msgs::env_data& msg);
void collect_heading(const bbauv_msgs::compass_data& msg);
void collect_velocity(const nav_msgs::Odometry::ConstPtr& msg);
void dynamic_reconfigure_callback(aggregator::controller_paramConfig &config, uint32_t level); 

/* ROS Initialization */

bbauv_msgs::controller_input navigation;
bbauv_msgs::controller_input tracking;
bbauv_msgs::controller_input ctrl;
bbauv_msgs::controller_onoff mode;
bbauv_msgs::controller_translational_constants trans_const;
bbauv_msgs::controller_rotational_constants rot_const;

ros::Publisher controller_input_pub;
ros::Publisher controller_mode_pub;
ros::Publisher controller_trans_const_pub;
ros::Publisher controller_rot_const_pub;

ros::Subscriber move_base_sub;
ros::Subscriber depth_base_sub;
ros::Subscriber tracker_sub;

ros::Subscriber depth_sub; 
ros::Subscriber compass_sub;
ros::Subscriber velocity_sub; 

/************ Main Loop *****************/

int main(int argc,char** argv) {

  ros::init(argc,argv,"aggregator");
  ros::NodeHandle nh;
 
  //subscribers declaration
  move_base_sub = nh.subscribe("/cmd_vel",20,update_move_base_setpoint,ros::TransportHints().tcpNoDelay());
  depth_base_sub = nh.subscribe("/cmd_depth",20,update_depth_base_setpoint,ros::TransportHints().tcpNoDelay());
  tracker_sub = nh.subscribe("/line_follower",20,update_vision_tracker_setpoint,ros::TransportHints().tcpNoDelay());
  depth_sub = nh.subscribe("/env_data",20,collect_depth,ros::TransportHints().tcpNoDelay());
  compass_sub = nh.subscribe("/os5000_data",20,collect_heading,ros::TransportHints().tcpNoDelay());
  velocity_sub = nh.subscribe("/odom",20,collect_velocity,ros::TransportHints().tcpNoDelay());
  
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
  ros::Rate loop_rate(16);
  while (ros::ok()) {

    //get Parameters from Param Server
        //Due to the use of an absolute pressure sensor, we need to subtract the pressure at atm bef we enter the water in order to obtain accurate depths
    nh.getParam("/aggregator/depthAtSurface", depthAtSurface); 
        //Are we IN TOPSIDE?
    nh.getParam("/aggregator/inTopside", inTopside);
        //Are we in STATE MACHINE?
    nh.getParam("/aggregator/inStateMachine", inStateMachine);
        //Are we in NAVIGATION?
    nh.getParam("/aggregator/inNavigation", inNavigation);
        //Are we in TRACKING?
    nh.getParam("/aggregator/inVisionTracking", inVisionTracking);

    if(inStateMachine)
    {
        if(!inTopside)
        {
            mode.topside=false;
        }
        if(inTopside)
        {
            mode.topside=true;
        }
    }

    if(inNavigation)
    {
      ctrl.depth_setpoint=navigation.depth_setpoint;
      ctrl.heading_setpoint=navigation.heading_setpoint;
      ctrl.forward_setpoint=navigation.forward_setpoint;
      ctrl.sidemove_setpoint=navigation.sidemove_setpoint;
    }
    if(inVisionTracking)
    {
      ctrl.depth_setpoint=tracking.depth_setpoint;
      ctrl.heading_setpoint=tracking.heading_setpoint;
      ctrl.forward_setpoint=tracking.forward_setpoint;
      ctrl.sidemove_setpoint=tracking.sidemove_setpoint;
    }

    controller_input_pub.publish(ctrl);
    controller_mode_pub.publish(mode);
    controller_trans_const_pub.publish(trans_const);
    controller_rot_const_pub.publish(rot_const);   

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/* ROS Callback functions */

//Setpoints
void update_move_base_setpoint(const geometry_msgs::Twist sp)
{
  navigation.heading_setpoint= sp.angular.z;
  navigation.forward_setpoint=sp.linear.x;
  navigation.sidemove_setpoint=sp.linear.y;
}

void update_depth_base_setpoint(const bbauv_msgs::controller_input depth)
{
  navigation.depth_setpoint=depth.depth_setpoint;
}

void update_vision_tracker_setpoint(const bbauv_msgs::controller_input track)
{
  tracking.depth_setpoint=track.depth_setpoint;
  tracking.heading_setpoint=track.heading_setpoint;
  tracking.forward_setpoint=track.forward_setpoint;
  tracking.sidemove_setpoint=track.sidemove_setpoint;
}

//Inputs from Sensors
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
  ctrl.depth_setpoint=config.depth_setpoint;

  rot_const.heading_kp=config.heading_kp;
  rot_const.heading_ki=config.heading_ki;
  rot_const.heading_kd=config.heading_kd;
  ctrl.heading_setpoint=config.heading_setpoint;
	
  trans_const.forward_kp=config.forward_kp;
  trans_const.forward_ki=config.forward_ki;
  trans_const.forward_kd=config.forward_kd;
  ctrl.forward_setpoint=config.forward_setpoint;
  //ctrl.forward_input=config.forward_input;

  trans_const.sidemove_kp=config.sidemove_kp;
  trans_const.sidemove_ki=config.sidemove_ki;
  trans_const.sidemove_kd=config.sidemove_kd;
  ctrl.sidemove_setpoint=config.sidemove_setpoint;

  mode.depth_PID=config.depth_PID;
  mode.heading_PID=config.heading_PID;
  mode.forward_PID=config.forward_PID;
  mode.sidemove_PID=config.sidemove_PID;
  mode.topside = config.topside;
  mode.teleop=config.teleop;

}

