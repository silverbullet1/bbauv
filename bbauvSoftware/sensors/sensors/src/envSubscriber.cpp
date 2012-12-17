#include "ros/ros.h"
#include "bbauv_msgs/env_data.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void msgprint(const bbauv_msgs::env_data::ConstPtr& msg)
{
    float Temp0 = msg->Temp0;
    float Temp1 = msg->Temp1;
    float Temp2 = msg->Temp2;
    float Depth  = msg->Depth;
    float WaterDetA = msg->WaterDetA;
    float WaterDetB = msg->WaterDetB;
    float WaterDetC = msg->WaterDetC;

    ROS_INFO("Temp0, Temp1, Temp2: [%lf %lf %lf]", Temp0, Temp1, Temp2);
    ROS_INFO("Depth: [%lf]", Depth);
    ROS_INFO("Water (A, B, C): [%lf %lf %lf]", WaterDetA, WaterDetB, WaterDetC);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "envSubscriber");

  ros::NodeHandle n;

  ros::Subscriber topicname_sub = n.subscribe("env_data", 1000, msgprint);

  ros::spin();

  return 0;
}
