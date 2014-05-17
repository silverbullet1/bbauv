#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <serial/serial.h>
#include <boost/algorithm/string.hpp>

#include <bbauv_msgs/imu_data.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#define AHRS8_ACC_BIAS 0.023
#define AHRS8_GYR_BIAS 10.8
#define AHRS8_ORI_BIAS 0.1

using std::string;
using std::cout;
using std::endl;

sig_atomic_t sigstatus = 0;

string stop("printtrigger 0 set drop\r\n");
string mask("printmask time_trigger yawt_trigger or pitch_trigger or roll_trigger or accelp_trigger or gyrop_trigger or quat_trigger or temp_trigger or set drop\r\n");
string rate("printmodulus 50 set drop\r\n");
string start("printtrigger printmask set drop\r\n");


void sigcallback(int sig)
{
    fprintf(stderr, "Signal caught: %d", sig);
    sigstatus = 1;
}

bool serialOK(string &st)
{
    boost::trim(st);
    return boost::ends_with(st, "OK");
}


void setup(serial::Serial* dev)
{
    string status;

    dev->write(stop);
    dev->write(mask);
    dev->write(rate);
    dev->write(start);
    dev->flush();
    ROS_INFO("SETUP OK");
}

string serialRead(serial::Serial *dev)
{
    string d;
    dev->readline(d, 512, string("\r\n"));
    if(boost::starts_with(d, "P:")){
        //ROS_INFO("data: %s", d.c_str());
        return d;
    } else{
        return serialRead(dev);
    }
}

void extractData(string d, bbauv_msgs::imu_data *imdata,
                 sensor_msgs::Imu *imdata_q,
                 float gyroMultiplier, std_msgs::Float32 *ftemp)
{

    std::vector<string> tokens;

    //perform sanity checks
    boost::trim(d);
    boost::erase_all(d, " ");
    boost::split(tokens, d, boost::is_any_of(","));

    if(tokens.size() != 23)
        return;

    unsigned int timestamp;
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float yawt, pitch, roll;
    float qw, qx, qy, qz;
    float temp;

    timestamp                       = atoi(tokens.at(1).c_str());
    Ax                              = atof(tokens.at(3).c_str()) / 1000.0;
    Ay                              = atof(tokens.at(4).c_str()) / 1000.0;
    Az                              = atof(tokens.at(5).c_str()) / 1000.0;
    Gx                              = atof(tokens.at(7).c_str()) * gyroMultiplier;
    Gy                              = atof(tokens.at(8).c_str()) * gyroMultiplier;
    Gz                              = atof(tokens.at(9).c_str()) * gyroMultiplier;
    yawt                            = atof(tokens.at(11).c_str());
    qw                              = atof(tokens.at(13).c_str());
    qx                              = atof(tokens.at(14).c_str());
    qy                              = atof(tokens.at(15).c_str());
    qz                              = atof(tokens.at(16).c_str());
    temp                            = atof(tokens.at(18).c_str());
    pitch                           = atof(tokens.at(20).c_str());
    roll                            = atof(tokens.at(22).c_str());

    imdata->orientation.x           = roll;
    imdata->orientation.y           = pitch;
    imdata->orientation.z           = yawt;

    imdata->angular_velocity.x      = Gx;
    imdata->angular_velocity.y      = Gy;
    imdata->angular_velocity.z      = Gz;

    imdata->linear_acceleration.x   = Ax;
    imdata->linear_acceleration.y   = Ay;
    imdata->linear_acceleration.z   = Az;

    imdata_q->orientation.w         = qw;
    imdata_q->orientation.x         = qx;
    imdata_q->orientation.y         = qy;
    imdata_q->orientation.z         = qz;

    imdata_q->angular_velocity.x    = Gx;
    imdata_q->angular_velocity.y    = Gy;
    imdata_q->angular_velocity.z    = Gz;

    imdata_q->linear_acceleration.x = Ax;
    imdata_q->linear_acceleration.y = Ay;
    imdata_q->linear_acceleration.z = Az;



    ftemp->data = temp;
}

int main(int argc, char **argv)
{
    string port;
    int baudrate;

    ros::init(argc, argv, "IMU_node",
              ros::init_options::NoSigintHandler
              );

    signal(SIGINT, sigcallback);

    ros::NodeHandle nh("~");
    ros::Rate loop_rate(40);
    nh.param("port", port, string("/dev/tty.usbserial-FTFUUY6A"));
    nh.param("baud", baudrate, int(115200));

    serial::Serial IMU(port, baudrate,
                       serial::Timeout::simpleTimeout(500));
    ros::Publisher imu_pub = nh.advertise<bbauv_msgs::imu_data>
        ("AHRS8_data_e", 1);
    ros::Publisher imu_pub_q = nh.advertise<sensor_msgs::Imu>
        ("AHRS8_data_q", 1);
    ros::Publisher imu_temp = nh.advertise<std_msgs::Float32>
        ("AHRS8_Temp", 1);

    if(!IMU.isOpen()){
        fprintf(stderr, "Serial port not open.");
        exit(1);
    }

    setup(&IMU);

    bbauv_msgs::imu_data imdata;
    sensor_msgs::Imu imdata_q;
    std_msgs::Float32 temp;

    try{
        while(ros::ok() && !sigstatus){
            extractData(serialRead(&IMU), &imdata, &imdata_q,
                        1.0, &temp);
            imu_pub.publish(imdata);
            imu_pub_q.publish(imdata_q);
            imu_temp.publish(temp);
            loop_rate.sleep();
            ros::spinOnce();
        }
    } catch(std::exception &e){
        fprintf(stderr, "Exception: %s", e.what());
    }

    ROS_INFO("Shutting down node.");
    IMU.flushInput(); IMU.flushOutput();
    IMU.write(stop); IMU.write(stop);
    IMU.close();
    return 0;
}
