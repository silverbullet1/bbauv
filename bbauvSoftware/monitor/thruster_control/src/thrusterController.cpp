#include "ros/ros.h"
#include "bbauv_msgs/thruster.h"
#include "sensor_msgs/Joy.h"

//declare publish message
bbauv_msgs::thruster msg;

void cal_speed(const sensor_msgs::Joy::ConstPtr& joy)
{
  msg.speed3=joy->axes[1]*2500;
}

int main(int argc, char **argv)
{
  //initialize ROS
  ros::init(argc, argv, "motorController");
  ros::NodeHandle n;

  //subscribe another topic for control mission
  //ros:Subscriber taks_sub = n.subscribe("task",1000,execute_task); 
  ros::Subscriber joy_sub=n.subscribe<sensor_msgs::Joy>("joy",10,cal_speed);

  //publish message data to [topic]motor_controller:
  ros::Publisher motor_pub = n.advertise<bbauv_msgs::thruster>("motor_controller", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    motor_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

