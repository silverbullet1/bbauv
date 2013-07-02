#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
using namespace std;

void callBack(const sensor_msgs::Imu::ConstPtr& msg);
double max_z_ang_vel;
double max_x_ang_vel;

int main(int argc, char**argv)
{
    ros::init(argc, argv, "AHRS8_Q2E");
    ros::NodeHandle nh;

    max_z_ang_vel = 0;
    max_x_ang_vel = 0;

    ros::Subscriber sub = nh.subscribe("AHRS8_data_q", 1000, callBack);
    ros::spin();
    return 0;
}

void callBack(const sensor_msgs::Imu::ConstPtr& msg){
    double q0 = (msg->orientation).w;
    double q1 = (msg->orientation).x;
    double q2 = (msg->orientation).y;
    double q3 = (msg->orientation).z;
    double r,p,y;

    r = atan2(2 * (q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
    p = asin(2 * (q0*q2 - q3*q1));
    y = atan2(2 * (q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));

    y = 2. * M_PI + y;
    if (y >= 2.*M_PI) y-= 2. * M_PI;

    ROS_INFO("AHRS8_Q2E Call Back - Roll, Pitch, Yaw");
    printf("%06.2lf -- %06.2lf -- %06.2lf\n", r, p ,y);

    if (abs((msg->angular_velocity).z) > max_z_ang_vel) 
        max_z_ang_vel = abs((msg->angular_velocity).z);

    if (abs((msg->angular_velocity).x) > max_x_ang_vel) 
        max_x_ang_vel = abs((msg->angular_velocity).x);


    ROS_INFO("max x angular_velocity: %lf", max_x_ang_vel);
    ROS_INFO("max z angular_velocity: %lf", max_z_ang_vel);
}
