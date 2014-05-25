#ifndef DVL_H
#define DVL_H

#include <serial/serial.h>
#include <time.h>
#include <bbauv_msgs/imu_data.h>
#include <dvl/decoder.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

class Decoder;

class DVL{
    serial::Serial* dvl;
    serial::Serial* dvl_sensor;
    Decoder* decoder;

    std::string dvl_port;
    std::string dvl_sensor_port;
    int dvl_baud, dvl_timeout;
    int dvl_sensor_baud, dvl_sensor_timeout;
public:
    DVL(std::string dvlport, int dvlbaud, int dvltimeout,
        std::string dvlsensorport, int dvlsensorbaud, int dvlsensortimeout);
    ~DVL();

    void runOnce();
    void sendBreak();
    bool setup();
    void integrate();
    void test(std::string fname);
    void populatesensors(const bbauv_msgs::imu_data::ConstPtr&);
    void fillpose(nav_msgs::Odometry*);
    void fillalt(std_msgs::Float32*);

    float x, y, z;

    float vfwd, vport, vvert;
    float pitch, roll, yaw;
    float altitude;
    double currtime, oldtime;
    float salinity, temp, depth, speedofsound;

    float ovfwd, ovport, ovvert;
};

#endif /* DVL_H */
