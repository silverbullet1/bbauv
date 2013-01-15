#include <ros/ros.h>
#include <bbauv_msgs/manual_control.h>
#include <sensor_msgs/Joy.h>

bbauv_msgs::manual_control msg;
void navigate(const sensor_msgs::Joy::ConstPtr& joy)
{

  msg.x=joy->axes[1];
  msg.y=joy->axes[0];
  msg.z=joy->axes[3];
  msg.yaw=-joy->axes[2]; //need to check the turn direction
// Notice: in controllerCode. (+) direction is clock-wise

  //scaling for yaw:
  if(msg.yaw>0) msg.yaw=msg.yaw*1.0/0.24;
  if(msg.yaw>1) msg.yaw=1;

  ROS_INFO("x=%f y=%f z=%f yaw=%f",msg.x,msg.y,msg.z,msg.yaw);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyTranslater");
  ros::NodeHandle n;
  
  ros::Subscriber joy_sub=n.subscribe<sensor_msgs::Joy>("joy",10,navigate,ros::TransportHints().tcpNoDelay());

  ros::Publisher manualcontrol_pub = n.advertise<bbauv_msgs::manual_control>("monitor_controller", 1000);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    manualcontrol_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}


