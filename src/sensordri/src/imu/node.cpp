#include <boost/algorithm/string.hpp>
#include <imu/imu.h>
#include <string>
#include <stdlib.h>
#include <assert.h>
#include <bbauv_msgs/imu_data.h>
#include <bbauv_msgs/compass_data.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

#define GYRO_SAMPLE_RATE 100

void imu::IMU::onInit()
{
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    std::string name, port;
    int baudrate;
    int timeout;
    int r;
    
    priv_nh.param("name", name, std::string("/AHRS8"));
    priv_nh.param("port", port, std::string("/dev/ttyGEDC6"));
    priv_nh.param("baud", baudrate, int(115200));
    priv_nh.param("timeout", timeout, int(100));
    priv_nh.param("rate", r, int(20));
    
    dev.reset(new serial::Serial(port, baudrate,
                                 serial::Timeout::simpleTimeout(timeout)));

    dev_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&imu::IMU::devicePoll, this)));

    collect_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&imu::IMU::collect, this)));

    rate.reset(new ros::Rate(r));

    const char* printmask = "printmask time_trigger yawt_trigger or "
        "roll_trigger or pitch_trigger or temp_trigger or "
        "accelp_trigger or gyrop_trigger or magp_trigger or "
        "gyror_trigger or set drop\r\n";

    dev->write("printtrigger 0 set drop\r\n");
    dev->write(std::string(printmask));
    dev->write("printmodulus 1 set drop\r\n");
    dev->write("printtrigger printmask set drop\r\n");
    dev->flush();

    serial_out = priv_nh.advertise<std_msgs::String>(name + "_serial", 100);
    imu_data_out = priv_nh.advertise<sensor_msgs::Imu>
        (name + "_data_q", 100);
    euler_out = priv_nh.advertise<bbauv_msgs::compass_data>
        (name + "_euler", 100);

    oldtime = 0;
}

void imu::IMU::devicePoll()
{
    size_t available;
    while(!boost::this_thread::interruption_requested() &&
          dev->isOpen() && ros::ok()){
        if((available = dev->available()) > 0){
            buffer.append(dev->read(available));
            dev->flush();
        }
    }
}

std::string imu::IMU::extract()
{
    size_t first = buffer.find("P:");
    size_t last  = buffer.rfind("\n");
    if(first == buffer.npos)
        return "";
    if(last == first)
        return "";

    std::string e = buffer.substr(first, last - first);
    buffer = buffer.substr(last, last - first);
    return e;
}

void imu::IMU::parse(imu::sensor_data *packet, std::string data)
{

    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != 26)
        return;

    double time = strtof(tok.at(1).c_str(), NULL);
    packet->timestamp = time;
    ros::Time tstamp(time);

    assert(oldtime < time);

    packet->ros_timestamp = tstamp;

    packet->magp[0]     = strtof(tok.at(3).c_str(), NULL);
    packet->magp[1]     = strtof(tok.at(4).c_str(), NULL);
    packet->magp[2]     = strtof(tok.at(5).c_str(), NULL);

    packet->accelp[0]   = strtof(tok.at(7).c_str(), NULL);
    packet->accelp[1]   = strtof(tok.at(8).c_str(), NULL);
    packet->accelp[2]   = strtof(tok.at(9).c_str(), NULL);

    packet->gyror[0]    = strtof(tok.at(11).c_str(), NULL);
    packet->gyror[1]    = strtof(tok.at(12).c_str(), NULL);
    packet->gyror[2]    = strtof(tok.at(13).c_str(), NULL);

    packet->gyrop[0]    = strtof(tok.at(15).c_str(), NULL);
    packet->gyrop[1]    = strtof(tok.at(16).c_str(), NULL);
    packet->gyrop[2]    = strtof(tok.at(17).c_str(), NULL);

    packet->yawt        = strtof(tok.at(19).c_str(), NULL);
    packet->temperature = strtof(tok.at(21).c_str(), NULL);
    packet->pitch       = strtof(tok.at(23).c_str(), NULL);
    packet->roll        = strtof(tok.at(25).c_str(), NULL);
}

void imu::IMU::process()
{
    int count = 0;
    float mean_yaw = 0, mean_roll = 0, mean_pitch = 0;
    float mean_temp = 0;
    float mean_accelp[3] = {0};
    float mean_gyrop[3] = {0};

    bbauv_msgs::compass_data euler_data;
    bbauv_msgs::imu_data imu_data;
    sensor_msgs::Imu ros_imu_data;

    double time = pqueue.back().timestamp;

    while(!pqueue.empty()){
        count++;
        sensor_data tmp = pqueue.front();
        //ROS_INFO("%f %f %f", tmp.yawt, tmp.pitch, tmp.roll);

        mean_yaw += tmp.yawt;
        mean_pitch += tmp.pitch;
        mean_roll += tmp.roll;
        mean_temp += tmp.temperature;
        
        for(size_t i = 0; i < 3; i++)
        {
            mean_accelp[i] += tmp.accelp[i];
            mean_gyrop[i] += tmp.gyrop[i];
        }

        for(float &x : tmp.accelp){
            x /= 1000;
        }

        for(float &x : tmp.gyrop){
            x *= GYRO_SAMPLE_RATE;
        }

        pqueue.pop();
    }

    mean_yaw /= count;
    mean_roll /= count;
    mean_pitch /= count;
    mean_temp /= count;

    for(size_t i = 0; i < 3; i++)
    {
        mean_accelp[i] /= count;
        mean_gyrop[i] /= count;
    }

    ROS_INFO("Filtered: %f %f %f", mean_yaw, mean_pitch, mean_roll);

    euler_data.roll = mean_roll;
    euler_data.pitch = mean_pitch;
    euler_data.yaw = mean_yaw;
    euler_data.Ax = mean_accelp[0];
    euler_data.Ay = mean_accelp[1];
    euler_data.Az = mean_accelp[2];
    euler_data.ang_vel_z = mean_gyrop[2];
    euler_data.temperature = mean_temp;

    imu_data.sens_covariance[0] = 0.01;
    imu_data.sens_covariance[4] = 0.01;
    imu_data.sens_covariance[8] = 0.01;

    ros_imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            radians(mean_roll), radians(mean_pitch),
            radians(mean_yaw)
        );

    ros_imu_data.orientation_covariance = imu_data.sens_covariance;
    ros_imu_data.angular_velocity.x = mean_gyrop[0];
    ros_imu_data.angular_velocity.y = mean_gyrop[1];
    ros_imu_data.angular_velocity.z = mean_gyrop[2];
    ros_imu_data.angular_velocity_covariance = imu_data.sens_covariance;
    ros_imu_data.linear_acceleration.x = mean_accelp[0];
    ros_imu_data.linear_acceleration.y = mean_accelp[1];
    ros_imu_data.linear_acceleration.z = mean_accelp[2];
    ros_imu_data.linear_acceleration_covariance = 
        imu_data.sens_covariance;
    ros_imu_data.header.stamp = ros::Time(time);

    if(imu_data_out)
        imu_data_out.publish(ros_imu_data);
    if(euler_out)
        euler_out.publish(euler_data);
}

void imu::IMU::collect()
{
    std_msgs::String out_debug;
    ROS_INFO("Collecting data from buffer.");
    while(!boost::this_thread::interruption_requested() && ros::ok())
    {
        std::string data = extract();
        boost::erase_all(data, " ");
        if(data.length() == 0){
            ROS_INFO("No useful data collected in spin.");
        } else{
            out_debug.data = data;
            if(serial_out)
                serial_out.publish(out_debug);

            std::vector<std::string> tok;
            boost::split(tok, data, boost::is_any_of("\n"));
            for(std::string line : tok){
                boost::trim(line);
                sensor_data packet;
                parse(&packet, line);
                pqueue.push(packet);
            }

            process();
        }


        ros::spinOnce();
        rate->sleep();
    }
}
