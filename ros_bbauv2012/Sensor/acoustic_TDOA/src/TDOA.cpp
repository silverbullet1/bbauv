#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
using namespace std;

// function headers
void near_field();
void far_field();

// variables
double d10, d20, d30;
double lamda;

// node constants
double x_1, y_1, z_1;
double x_2, y_2, z_2;
double x_3, y_3, z_3;
double speedOfSound;

// input from outside
double phase1 = 0;
double phase2 = 0;
double phase3 = 0;
int freq = 20000;
bool nearField = true;

// output data
double x, y, z;

void acoustic_callback(const geometry_msgs::Twist::ConstPtr& msg);
//void control_callback(const );

int main(int argc, char **argv){
    //topic names
    string acoustic_topic = "/hydrophone/phase_diff"; // input topic from the sonar system
    string control_topic = "control_TDOA";            // input topic from the top side control (z and mode)
    string TDOA_topic = "acoustic_TDOA";              // output topic

    // initialize node constants
    int ros_rate = 10;
    speedOfSound = 1484;
    x_1 = 0.04; y_1 = 0;    z_1 = 0;
    x_2 = 0;    y_2 = 0.04; z_2 = 0;
    x_3 = 0;    y_3 = 0;    z_3 = 0.04;
    z = 10000;
    
    ros::init(argc, argv, "acoustic_TDOA");
    ros::NodeHandle nh;
    geometry_msgs::Point point_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>(TDOA_topic, 1000);
    ros::Subscriber sub = nh.subscribe(acoustic_topic, 1000, acoustic_callback);

    ros::Rate loop_rate(ros_rate);
    while (ros::ok()){
        lamda = speedOfSound * 1.0 / freq;
        d10 = phase1 * lamda * 1.0 / (2 * M_PI);
        d20 = phase2 * lamda * 1.0 / (2 * M_PI);
        d30 = phase3 * lamda * 1.0 / (2 * M_PI);
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

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void near_field(){
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

    
    cout << "debug: A2 _ B2 _ C2: "
         << A2 << " _ "
         << B2 << " _ "
         << C2 << endl;
    cout << "debug: A3 _ B3 _ C3: "
         << A3 << " _ "
         << B3 << " _ "
         << C3 << endl;
    

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

void acoustic_callback(const geometry_msgs::Twist::ConstPtr& msg){
    phase1 = (msg->linear).x; 
    phase2 = (msg->linear).y;
    phase3 = (msg->linear).z;
    freq = (msg->angular).x;
}
