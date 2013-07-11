#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;

int main(int argc, char **argv){
    ros::init(argc,argv,"TDOA_test_stub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/hydrophone/phase_diff", 1000);
    ros::Rate loop_rate(10);
    
    double xa = 0.04, ya = 0,    za = 0;
    double xb = 0,    yb = 0.04, zb = 0;
    double xc = 0,    yc = 0,    zc = 0.04;
    int soundSpeed = 1484;
    int freq = 20000;
    double lamda = soundSpeed * 1.0 / freq;

    double x = 10000;
    double y = 10000;
    double z = 10000;

    double d10 = sqrt(pow(x-xa,2)+pow(y-ya,2)+pow(z-za,2));
    d10 -= sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    double d20 = sqrt(pow(x-xb,2)+pow(y-yb,2)+pow(z-zb,2));
    d20 -= sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    double d30 = sqrt(pow(x-xc,2)+pow(y-yc,2)+pow(z-zc,2));
    d30 -= sqrt(pow(x,2) + pow(y,2) + pow(z,2));

    double phase1 = 2.0 * M_PI * d10 / lamda;
    double phase2 = 2.0 * M_PI * d20 / lamda;
    double phase3 = 2.0 * M_PI * d30 / lamda;

    geometry_msgs::Twist twist_msg;
    while (ros::ok()){
        twist_msg.linear.x = phase1;
        twist_msg.linear.y = phase2;
        twist_msg.linear.z = phase3;
        twist_msg.angular.x = freq;

        pub.publish(twist_msg);

        loop_rate.sleep();
    }
}
