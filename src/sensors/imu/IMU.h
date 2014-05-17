#ifndef IMU_H
#define IMU_H

#include <serial/serial.h>
#include <bbauv_msgs/imu_data.h>
#include <std_msgs/Float32.h>

class IMU
{
public:
    serial::Serial *dev;
    bbauv_msgs::imu_data imu_data;
    std_msgs::Float32 temperature;

    IMU(std::string port, int baud, int timeout);
    ~IMU();

    bool setup();
    std::string read();
    void process();

};

#endif
