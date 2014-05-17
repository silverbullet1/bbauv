#include "IMU.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv)
{
    std::string port;
    int baudrate;
    int timeout;

    ros::init(argc, argv, "IMU_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(40);

    nh.param("port", port, std::string("/dev/ttyAHRS"));
    nh.param("baud", baudrate, int(115200));
    nh.param("timeout", timeout, int(500));

    ros::Publisher imu_pub = nh.advertise<bbauv_msgs::imu_data>
        ("/AHRS8_data_e", 1);
    ros::Publisher imu_temp = nh.advertise<std_msgs::Float32>
        ("/AHRS8_Temp", 1);

    IMU ahrs8(port, baudrate, timeout);
    ahrs8.setup();
    while(ros::ok()){
        ahrs8.process();
        imu_pub.publish(ahrs8.imu_data);
        imu_temp.publish(ahrs8.temperature);
        ros::spinOnce();
    }

    return 0;
}
