#ifndef DVL_H
#define DVL_H

#include <string>
#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include "decoder.h"
#include <bbauv_msgs/imu_data.h>
#include <dvl/dvlConfig.h>

class PD6Decoder;

class DVL{
    std::string port;
    int baudrate, timeout;
    serial::Serial* dvl;

public:

    int counter;

    float north, east, up;
    float veast, vnorth, vup;
    float oldveast, oldvnorth, oldvup;
    float lasttime, currtime;
    float e_north, e_east, e_up, old_e_north, old_e_east,
          old_e_up;
    float ex_n, ex_e, ex_u;
    float yaw;
    bool debug;

    PD6Decoder* decoder;
    DVL(std::string, int, int);
    ~DVL();
    bool setup();
    void poll();
    void integrate();
    void collect(ros::Publisher*, ros::Publisher*);
    void AHRSsub(const bbauv_msgs::imu_dataConstPtr &imu);
    void zero(dvl::dvlConfig &config, uint32_t level);
    void zero_relative(dvl::dvlConfig &config, uint32_t level);

};

#endif /* DVL_H */
