#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <bbauv_msgs/acoustic_control.h>
using namespace std;

// function headers
void near_field();
void far_field();

// position of the mount in relative to the base_footprint
double x_offset;
double y_offset;
double z_offset;

// variables
double d10, d20, d30;
double lamda;

// node constants
double x_1, y_1, z_1;
double x_2, y_2, z_2;
double x_3, y_3, z_3;
double speedOfSound;

// input from outside
double time_diff1 = 0;
double time_diff2 = 0;
double time_diff3 = 0;
int freq = 30000;
bool nearField = true;

// output data
double x, y, z;

void hydrophone_callback(const geometry_msgs::Twist::ConstPtr& msg);
void control_callback(const bbauv_msgs::acoustic_control::ConstPtr& msg);

int main(int argc, char **argv){
    //topic names
    string hydrophone_topic = "/hydrophone/time_diff"; // input topic from the sonar system
    string control_topic = "control_TDOA";           // input topic from the top side control (z and mode)
    string TDOA_topic = "acoustic_TDOA";             // output topic

    // initialize node constants
    int ros_rate = 10;
    speedOfSound = 1484;
    x_1 = 0.07;    y_1 = 0.07;    z_1 = 0;
    x_2 = 0.07;    y_2 = -0.07;   z_2 = 0;
    x_3 = 0;  	   y_3 = 0;       z_3 = -0.1;
    z = 3;
    
    ros::init(argc, argv, "acoustic_TDOA");
    ros::NodeHandle nh;
    geometry_msgs::Point point_msg;
    bbauv_msgs::acoustic_control control_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>(TDOA_topic, 100);
    ros::Subscriber hydrophone_sub = nh.subscribe(hydrophone_topic, 100, hydrophone_callback);
    ros::Subscriber control_sub = nh.subscribe(control_topic, 100, control_callback);

    ros::Rate loop_rate(ros_rate);
    while (ros::ok()){
        d10 = time_diff1 * speedOfSound;
        d20 = time_diff2 * speedOfSound;
        d30 = time_diff3 * speedOfSound;
        /*cout << "debug: d10 _ d20 _ d30: " 
            << d10 << " _ "
            << d20 << " _ "
            << d30 << endl;
        */

        if (nearField) near_field();
        else far_field();

        point_msg.x = x;
        point_msg.y = y;
        point_msg.z = z;
        pub.publish(point_msg);
	cout << "YAW:" << atan2(y,x) * 180.0 / M_PI << endl;;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void near_field(){
    if (d10 == 0 || d20 == 0 || d30 == 0) 
        return;

    double A2 = 2 * (x_2 / d20 - x_1 / d10);
    double B2 = 2 * (y_2 / d20 - y_1 / d10);
    double A3 = 2 * (x_3 / d30 - x_1 / d10);
    double B3 = 2 * (y_3 / d30 - y_1 / d10);
    
    double C2 = z * 2 * (z_2 * 1.0/d20 - z_1 * 1.0/d10);
    C2 += d20 - d10;
    C2 += -(x_2 * x_2 + y_2 * y_2 + z_2 * z_2)/d20 + (x_1 * x_1 + y_1 * y_1 + z_1 * z_1)/d10;
    C2 = -C2;
    double C3 = z * 2 * (z_3 * 1.0/d30 - z_1 * 1.0/d10);
    C3 += d30 - d10 ;
    C3 += -(x_3 * x_3 + y_3 * y_3 + z_3 * z_3)/d30 + (x_1 * x_1 + y_1 * y_1 + z_1 * z_1)/d10;
    C3 = -C3;

    if (A2*B3 == A3*B2) {
        x = 0;
        y = 0;
    }
    else {
        x = (C2 * B3 - C3 * B2) / (A2 * B3 - A3 * B2);
        y = (C3 * A2 - C2 * A3) / (B3 * A2 - B2 * A3);
    }
}

void far_field(){
    double detA = x_1 * (y_2 * z_3 - y_3 * z_2);
    detA -= y_1 * (x_2 * z_3 - x_3 * z_2);
    detA += z_1 * (x_2 * y_3 - x_3 * y_2);
    
    //cout << "debug: detA : " << detA << endl;

    x  = d10 * (y_2 * z_3 - y_3 * z_2);
    x -= d20 * (y_1 * z_3 - y_3 * z_1);
    x += d30 * (y_1 * z_2 - y_2 * z_1);

    y = -d10 * (x_2 * z_3 - x_3 * z_2);
    y += d20 * (x_1 * z_3 - x_3 * z_1);
    y -= d30 * (x_1 * z_2 - x_2 * z_1);

    z  = d10 * (x_2 * y_3 - x_3 * y_2);
    z -= d20 * (x_1 * y_3 - x_3 * y_1);
    z += d30 * (x_1 * y_2 - x_2 * y_1);

    x = x / detA;
    y = y / detA;
    z = z / detA;

    double nor = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    x = x / nor;
    y = y / nor;
    z = z / nor;
}

void hydrophone_callback(const geometry_msgs::Twist::ConstPtr& msg){
    time_diff1 = (msg->linear).x * 1.0 / 1000000; 
    time_diff2 = (msg->linear).y * 1.0 / 1000000;
    time_diff3 = (msg->linear).z * 1.0 / 1000000;
    freq = (msg->angular).x;
}

void control_callback(const bbauv_msgs::acoustic_control::ConstPtr& msg) {
    z = msg->z;
    nearField = msg->nearField;
}
