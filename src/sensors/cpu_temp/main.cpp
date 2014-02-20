#include <stdio.h>
#include <ros/ros.h>
#include <bbauv_msgs/cpu_temperature.h>
#include <sensors/sensors.h>

int main(int argc, char **argv)
{
    sensors_init(NULL);
    bbauv_msgs::cpu_temperature cpu_temperature;
    
    ros::init(argc, argv, "cpu_temperature");
    while(ros::ok()){
        ros::spinOnce();
    }
}
