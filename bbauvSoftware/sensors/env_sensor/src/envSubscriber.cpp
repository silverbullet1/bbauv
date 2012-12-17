#include "ros/ros.h"
#include "bbauv_msgs/env_data.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const bbauv_msgs::env_data::ConstPtr& msg)
{
    double T0 = msg->Temp0;
    double T1 = msg->Temp1;
    double T2 = msg->Temp2;
    double D  = msg->Depth;
    double WDa = msg->WaterDetA;
    double WDb = msg->WaterDetB;
    double WDc = msg->WaterDetC;

    ROS_INFO("T0, T1, T2: [%lf %lf %lf]", T0, T1, T2);
    ROS_INFO("D: [%lf]", D);
    ROS_INFO("W (a, b, c): [%lf %lf %lf]", WDa, WDb, WDc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "envSubscriber");

  ros::NodeHandle n;

  ros::Subscriber topicname_sub = n.subscribe("env_data", 1000, topicnameCallback);

  ros::spin();

  return 0;
}
