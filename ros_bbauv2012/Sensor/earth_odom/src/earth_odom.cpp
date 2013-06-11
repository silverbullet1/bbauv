#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <bbauv_msgs/imu_data.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <earth_odom/earth_odomConfig.h>
using namespace std;

double angXvel, angYvel, angZvel;
double angX, angY, angZ;
double angXvelcov, angYvelcov, angZvelcov;
double angXcov, angYcov, angZcov;
double xvel, yvel, zvel;
double Nvel, Evel, Dvel;
double lastNvel, lastEvel, lastDvel;
double north, east, down;
double Ncov, Ecov, Dcov;
double Nvelcov, Evelcov, Dvelcov;
double lastSec, curSec;
double startSec;

const string imu_topic    = "AHRS8_data_e";
const string dvl_topic    = "WH_DVL_data";
const string output_topic = "earth_odom";
const string header_frame_id = "odom";
const string child_frame_id = "base_footprint";

void imu_callback(const bbauv_msgs::imu_data::ConstPtr& imu_msg);
void dvl_callback(const nav_msgs::Odometry::ConstPtr& dvl_msg);
void config_callback(earth_odom::earth_odomConfig &config, uint32_t level);
void publish_msg(ros::Publisher *pubData);
void zero_distance();


int main(int argc, char **argv){
    ros::init(argc, argv, "earth_odom");

    // node variables initiation
    angXvel = 0; angYvel = 0; angZvel = 0;
    angX = 0; angY = 0; angZ = 0;
    angXvelcov = 0.1; angYvelcov = 0.1; angZvelcov = 0.1;
    angXcov = 0.1; angYcov = 0.1; angZcov = 0.1;
    zero_distance();
    lastSec = 0; curSec = 0;
    
    
    // ROS initiation
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1000, imu_callback);
    ros::Subscriber dvl_sub = nh.subscribe(dvl_topic, 1000, dvl_callback);
    ros::Publisher pubData = nh.advertise<nav_msgs::Odometry>(output_topic, 1000);
    ros::Rate loop_rate(20);
    dynamic_reconfigure::Server<earth_odom::earth_odomConfig> server;
    dynamic_reconfigure::Server<earth_odom::earth_odomConfig>::CallbackType f;
    f = boost::bind(&config_callback, _1, _2);
    server.setCallback(f);

    while(ros::ok()){
        ros::spinOnce();
        publish_msg(&pubData);
        loop_rate.sleep();
    }
    
    return 0;
}

void imu_callback(const bbauv_msgs::imu_data::ConstPtr& imu_msg){
    angX = (imu_msg->orientation).x;
    angY = (imu_msg->orientation).y;
    angZ = (imu_msg->orientation).z;

    angXvel = (imu_msg->angular_velocity).x;
    angYvel = (imu_msg->angular_velocity).y;
    angZvel = (imu_msg->angular_velocity).z;
}

void dvl_callback(const nav_msgs::Odometry::ConstPtr& dvl_msg){
    lastSec = curSec;
    curSec = (dvl_msg->header).stamp.toSec();
    if (startSec == 0) startSec = curSec;
    if (lastSec == 0) lastSec = curSec;
    
    lastNvel = Nvel;
    lastEvel = Evel;
    lastDvel = Dvel;

    xvel = (dvl_msg->twist).twist.linear.x;
    yvel = (dvl_msg->twist).twist.linear.y;
    zvel = (dvl_msg->twist).twist.linear.z;

    Nvel = xvel * cos(angZ) - yvel * sin(angZ);
    Evel = xvel * sin(angZ) + yvel * cos(angZ);
    Dvel = zvel; 
    
    if (abs(Nvel) > 5 || abs(Evel) > 5 || abs(Dvel) > 5 ||
        abs(lastNvel) > 5 || abs(lastEvel) > 5 || abs(lastDvel) > 5){
        ROS_INFO("earth_odom::bad velocity");
        return;
    }

    // integrate for distances
    north += (lastNvel + Nvel) * (curSec - lastSec) / 2.0;
    east  += (lastEvel + Evel) * (curSec - lastSec) / 2.0;
    down  += (lastDvel + Dvel) * (curSec - lastSec) / 2.0;

    // convert velocity covariance
    Nvelcov = (dvl_msg->twist).covariance[0];
    Evelcov = (dvl_msg->twist).covariance[7];
    Dvelcov = (dvl_msg->twist).covariance[14];

    // compute distance covariance
    Ncov = Nvelcov * (curSec - startSec);
    Ecov = Evelcov * (curSec - startSec);
    Dcov = Dvelcov * (curSec - startSec);

    return;
}

void config_callback(earth_odom::earth_odomConfig &config, uint32_t level){
    if (config.zero_distance == true){
        zero_distance();
        ROS_INFO("earth_odom::zero_distance");
        config.zero_distance = false;
    }
}

void publish_msg(ros::Publisher *pubData){
    nav_msgs::Odometry odomData;

    //time stamp
    //ros time starts in 1970
    ros::Time givenTime(curSec);
    odomData.header.stamp = givenTime;

    // coordinate frames
    odomData.header.frame_id = header_frame_id;
    odomData.child_frame_id  = child_frame_id;

    // FILL IN POSE DATA
    // position
    odomData.pose.pose.position.x = north;
    odomData.pose.pose.position.y = east;
    odomData.pose.pose.position.z = down;

    // orientation
    odomData.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angX, angY, angZ);
    /*ROS_DEBUG("DVL quaternions = %.1f, %.1f, %.1f, %.1f", 
            odomData.pose.pose.orientation.x, 
            odomData.pose.pose.orientation.y, 
            odomData.pose.pose.orientation.z, 
            odomData.pose.pose.orientation.w);
    */

    // pose covariance matrix
    for ( int i=0; i < 36; i++)
        odomData.pose.covariance[i] = 0;
    
    odomData.pose.covariance[0]  = Ncov;
    odomData.pose.covariance[7]  = Ecov;
    odomData.pose.covariance[14] = Dcov;
    odomData.pose.covariance[21] = angXcov;
    odomData.pose.covariance[28] = angYcov;
    odomData.pose.covariance[35] = angZcov;


    // FILL IN TWIST DATA
    // twist covariance matrix
    for ( int i=0; i < 36; i++)
        odomData.twist.covariance[i] = 0;
    
    odomData.twist.covariance[0]  = Nvelcov;
    odomData.twist.covariance[7]  = Evelcov;
    odomData.twist.covariance[14] = Dvelcov;
    odomData.twist.covariance[21] = angXvelcov;
    odomData.twist.covariance[28] = angYvelcov;
    odomData.twist.covariance[35] = angZvelcov;

    odomData.twist.twist.angular.x = angXvel;
    odomData.twist.twist.angular.y = angYvel;
    odomData.twist.twist.angular.z = angZvel;

    odomData.twist.twist.linear.x  = Nvel;
    odomData.twist.twist.linear.y  = Evel;
    odomData.twist.twist.linear.z  = Dvel;

    pubData->publish(odomData);
}

void zero_distance(){
    lastNvel = 0;
    lastEvel = 0;
    lastDvel = 0;
    Nvel = 0; Evel = 0; Dvel = 0;
    east = 0; north = 0; down = 0;
    Ncov = 0; Ecov = 0; Dcov = 0;
    Nvelcov = 0; Evelcov = 0; Dvelcov = 0;
    startSec = 0;
}
