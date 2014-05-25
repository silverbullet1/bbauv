#ifndef EXPLORER_H
#define EXPLORER_H

#include <serial/serial.h>
#include <bbauv_msgs/imu_data.h>

class DVL
{
    std::string port;
    int baudrate;
    int timeout;
    serial::Serial *dev;
    serial::Serial *sensors;
public:
    DVL(std::string, int, int, std::string,
        int, int);
    ~DVL(){};
    void setup();
    void sendBreak();
    void populatesensors(bbauv_msgs::imu_data*);
    std::string read();

    double ltime;
    double time;
};

#endif
