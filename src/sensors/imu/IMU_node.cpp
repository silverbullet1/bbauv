#include "IMU.h"

#include <boost/algorithm/string.hpp>
#include <math.h>

#define DEBUG 0
#define GYRO_SAMPLE_RATE 100

/*
 *  Part of the ROS driver for our sparton AHRS-8 and GEDC-6
 *  -alex
 */


IMU::IMU(std::string port, int baud, int timeout)
{
    dev = new serial::Serial(port, baud,
                         serial::Timeout::simpleTimeout(timeout));
}

IMU::~IMU()
{
    dev->close();
    delete(dev);
}

bool IMU::setup()
{
    if(!dev->isOpen())
        return false;

    std::string stop("printtrigger 0 set drop\r\n");
    std::string mask("printmask time_trigger yawt_trigger or roll_trigger or pitch_trigger or accelp_trigger or gyrop_trigger or temp_trigger or set drop\r\n");
    std::string rate("printmodulus 2 set drop\r\n");
    std::string start("printtrigger printmask set drop\r\n");

    //imu_data = new bbauv_msgs::imu_data();
    //temperature = new std_msgs::Float32();

    if(dev->available()){
        if(true)
            fprintf(stderr, "The sensor was already streaming data.\n");

        /*
         * @TODO: can't stop the streaming for some reason.
         */
        return true;
    }

    dev->write(mask);
    if(true)
        fprintf(stderr, "%s", dev->readline().c_str());
    dev->write(rate);
    if(true)
        fprintf(stderr, "%s", dev->readline().c_str());
    dev->write(start);
    if(true)
        fprintf(stderr, "%s", dev->readline().c_str());

    return true;
}

std::string IMU::read()
{
    std::string data;
    data = dev->readline();
    boost::trim(data);
    boost::erase_all(data, " ");

    if(boost::starts_with(data, "P:")){
        //ROS_INFO("%s\n", data.c_str());
        //fprintf(stdout, "%s\n", data.c_str());
        return data;
    } else{
        return read();
    }
}

float degtorad(float deg)
{
    return deg * (M_PI / 180.0);
}

float radtodeg(float rad)
{
    return rad * (180 / M_PI);
} 

void IMU::process()
{
    std::string data = read();
    //if(DEBUG)
    //    fprintf(stdout, "%s\n", data.c_str());

    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));
    //if(DEBUG)
    //    fprintf(stdout, "%d\n", (int) tok.size());
    
    if(tok.size() != 18)
        return;

    //for(int i = 0; i < (int) tok.size(); i++)
    //    fprintf(stdout, "%d -> %s\n", i, tok.at(i).c_str());
    
    unsigned int timestamp = atoi(tok.at(1).c_str());
    float Ax               = atof(tok.at(3).c_str()) / 1000.0;
    float Ay               = atof(tok.at(4).c_str()) / 1000.0;
    float Az               = atof(tok.at(5).c_str()) / 1000.0;
    float Gx               = atof(tok.at(7).c_str()) * GYRO_SAMPLE_RATE;
    float Gy               = atof(tok.at(8).c_str()) * GYRO_SAMPLE_RATE;
    float Gz               = atof(tok.at(9).c_str()) * GYRO_SAMPLE_RATE;
    float yaw              = (atof(tok.at(11).c_str()));
    float temp             = atof(tok.at(13).c_str());
    float pitch            = (atof(tok.at(15).c_str()));
    float roll             = (atof(tok.at(17).c_str()));

    imu_data.orientation.z         = degtorad(yaw);
    imu_data.orientation.y         = degtorad(pitch);
    imu_data.orientation.x         = degtorad(roll);

    imu_data.linear_acceleration.x = Ax;
    imu_data.linear_acceleration.y = Ay;
    imu_data.linear_acceleration.z = Az;

    imu_data.angular_velocity.x    = Gx;
    imu_data.angular_velocity.y    = Gy;
    imu_data.angular_velocity.z    = Gz;

    temperature.data               = temp;

    euler.roll = roll;
    euler.pitch = pitch;
    euler.yaw = yaw;

    euler.temperature = temp;
    euler.Ax = Ax;
    euler.Ay = Ay;
    euler.Az = Az;

    if(DEBUG)
        fprintf(stderr, "%s\n", data.c_str());
}

