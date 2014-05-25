#include <dvl/dvl.h>
#include <ros/ros.h>
#include <bbauv_msgs/imu_data.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv){
    /*
    DVL d("/dev/tty.NoZAP-PL2303-00003014", 9600, 1500, "/dev/tty.usbserial-FTB3LPPT",
          9600, 1500);
    d.test("/Users/alex/dvl.cap");
    */

    ros::init(argc, argv, "DVL");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(20);

    DVL explorer("/dev/ttyDVL", 9600, 1500,
                 "/dev/ttyDVLSensor", 9600, 1500);

    if(!explorer.setup()){
        ROS_ERROR("Couldn't proc /dev/ttyDVL");
        exit(1);
    }

    ros::Subscriber sub = nh.subscribe("/AHRS8_data_e", 10,
                                       &DVL::populatesensors, &explorer);
    ros::Publisher posepub = nh.advertise<nav_msgs::Odometry>
        ("/WH_DVL_data", 100);
    ros::Publisher altpub = nh.advertise<std_msgs::Float32>
        ("/altitude", 100);

    nav_msgs::Odometry odomobject;
    std_msgs::Float32  altitudeobject;

    while(ros::ok()){
        explorer.runOnce();
        explorer.fillpose(&odomobject);
        explorer.fillalt(&altitudeobject);
        posepub.publish(odomobject);
        altpub.publish(altitudeobject);
        ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}
