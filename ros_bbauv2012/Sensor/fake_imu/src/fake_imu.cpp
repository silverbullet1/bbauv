#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace std;

const string in_topic  = "WH_DVL_data";
const string out_topic = "imu_data";
const string frameId   = "base_footprint";
const int rate = 4;

sensor_msgs::Imu imumsg;

void callBack(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_imu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(in_topic, 1000, callBack);
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(out_topic, 1000);
    for (int i=0;i<9;i++) {
        imumsg.orientation_covariance[i] = 0;
        imumsg.angular_velocity_covariance[i] = 0;
        imumsg.linear_acceleration_covariance[i] = 0;
    }

    ros::Rate loop_rate(rate);
    
    while (ros::ok()) {
        pub.publish(imumsg);
        ros::spinOnce();
        loop_rate.sleep();    
    }
}

void callBack(const nav_msgs::Odometry::ConstPtr& msg) {
    imumsg.header.stamp = (msg->header).stamp;
    imumsg.header.frame_id = frameId;

    imumsg.orientation.x = (msg->pose).pose.orientation.x;
    imumsg.orientation.y = (msg->pose).pose.orientation.y;
    imumsg.orientation.z = (msg->pose).pose.orientation.z;
    imumsg.orientation.w = (msg->pose).pose.orientation.w;

    imumsg.angular_velocity.x = (msg->twist).twist.angular.x;
    imumsg.angular_velocity.y = (msg->twist).twist.angular.y;
    imumsg.angular_velocity.z = (msg->twist).twist.angular.z;

    imumsg.linear_acceleration.x = 0.001;
    imumsg.linear_acceleration.x = 0.001;
    imumsg.linear_acceleration.x = 0.001;
    
    imumsg.orientation_covariance[0] = (msg->pose).covariance[21];
    imumsg.orientation_covariance[4] = (msg->pose).covariance[28];
    imumsg.orientation_covariance[7] = (msg->pose).covariance[35];

    imumsg.angular_velocity_covariance[0] = (msg->twist).covariance[21];
    imumsg.angular_velocity_covariance[4] = (msg->twist).covariance[28];
    imumsg.angular_velocity_covariance[7] = (msg->twist).covariance[35];
    imumsg.linear_acceleration_covariance[0] = 0.002;
    imumsg.linear_acceleration_covariance[4] = 0.002;
    imumsg.linear_acceleration_covariance[7] = 0.002;
}
