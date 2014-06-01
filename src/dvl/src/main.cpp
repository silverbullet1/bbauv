#include "../include/DVL.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <dvl/dvlConfig.h>
#include <functional>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DVL");
    ros::NodeHandle nh("~");

    std::string port;
    int baudrate;
    int timeout;
    int rosrate;

    nh.param("port", port, std::string("/dev/ttyDVL"));
    nh.param("baud", baudrate, int(9600));
    nh.param("timeout", timeout, int(1000));
    nh.param("rate", rosrate, int(7));

    ros::Rate loop_rate(rosrate);

    ros::Publisher dvlpub = nh.advertise<nav_msgs::Odometry>("/WH_DVL_data",
                                                             100);
    ros::Publisher earth_odom = nh.advertise<nav_msgs::Odometry>
        ("/earth_odom", 100);

    DVL dvl(port, baudrate, timeout);

    ros::Subscriber ahrs_sub = nh.subscribe("/AHRS8_data_e", 10,
                                            &DVL::AHRSsub, &dvl);
    dynamic_reconfigure::Server<dvl::dvlConfig> dynserver;
    dynamic_reconfigure::Server<dvl::dvlConfig>::CallbackType f;
    f = boost::bind(&DVL::zero, &dvl, _1, _2);
    dynserver.setCallback(f);

    if(!dvl.setup()){
        ROS_ERROR("[DVL] Cannot initialize DVL");
        nh.shutdown();
        ros::shutdown();
    }

    ROS_INFO("[DVL] Initialized on %s at %d bps.", port.c_str(), baudrate);
    while(ros::ok()){
        dvl.poll();
        dvl.integrate();
        dvl.collect(&dvlpub, &earth_odom);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
