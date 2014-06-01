#include <boost/algorithm/string.hpp>
#include <../include/DVL.h>
#include <nav_msgs/Odometry.h>
#include <bbauv_msgs/imu_data.h>
#include <stdexcept>


DVL::DVL(std::string serial_port, int baud, int tout)
{
    port = serial_port;
    baudrate = baud;
    timeout = tout;
}

bool DVL::setup()
{

    decoder = new PD6Decoder(this);
    counter = 0;
    north = east = up = 0;
    oldvup = oldveast = oldvnorth = 0;
    currtime = lasttime = 0;

    yaw = 0;
    debug = false;

    e_north = e_east = e_up = old_e_up = old_e_east =
        old_e_north = ex_n = ex_e = ex_u = 0;

    try{
        dvl = new serial::Serial(port, baudrate,
                                 serial::Timeout::simpleTimeout(timeout));
        if(!dvl->isOpen()){
            throw std::runtime_error(std::string("Serial port is not open"));
        }

        ROS_INFO("[DVL] Sending soft break.");
        dvl->write("+++\r\n");
        sleep(1);
        ROS_INFO("[DVL] Starting to ping.");
        dvl->write("CS\r\n");

        dvl->flushInput();
        dvl->flushOutput();
    } catch(std::exception &e){
        ROS_ERROR("Exception initializing: [%s]", e.what());
        return false;
    }

    return true;
}

void DVL::poll()
{

    std::string buffer;
    dvl->read(buffer, dvl->available());
    decoder->buffer.append(buffer);
    decoder->parse();
}

void DVL::integrate()
{
    while(!decoder->ensembles.empty()){
        //ROS_INFO("size of queue: %d", (int) decoder->ensembles.size());
        ensemble en = decoder->ensembles.front();
        if(en.status == false){
            ROS_ERROR("[DVL] Bottom track lost");
            decoder->ensembles.pop();
            return;
        }

        if(fabs(en.bv_up) > 10 || fabs(en.bv_east) > 10 ||
           fabs(en.bv_north) > 10)
        {
            ROS_ERROR("[DVL] Bad velocity");
            ROS_INFO("%f %f %f", en.bv_up, en.bv_east, en.bv_north);
            decoder->ensembles.pop();
            return;
        }

        old_e_north = e_north;
        old_e_up = e_up;
        old_e_east = e_east;

        oldveast = veast;
        oldvnorth = vnorth;
        oldvup = vup;

        veast = en.bv_east;
        vnorth = en.bv_north;
        vup = en.bv_up;

        //e_north = vnorth * cos(yaw) + veast * sin(yaw);
        //e_east = vnorth * sin(yaw) - veast * cos(yaw);
        e_north = veast * cos(yaw + M_PI) + vnorth * sin(yaw);
        e_east = veast * sin(yaw + M_PI) + vnorth * cos(yaw);
        e_up = vup;

        lasttime = currtime;
        currtime = en.timestamp;

        if(lasttime == 0)
            lasttime = currtime;

        if(lasttime > currtime){
            ROS_ERROR("[DVL] Timetravel, previous time more than current");
            decoder->ensembles.pop();
            return;
        }

        if(fabs(currtime - lasttime) > 120){
            ROS_ERROR("[DVL] Too long before last known bottom track");
            ROS_INFO("[DVL] curr: %f, last: %f", currtime, lasttime);
            decoder->ensembles.pop();
            return;
        }

        double delta = currtime - lasttime;

        north += (vnorth + oldvnorth) * delta / 2.0;
        east  += (veast + oldveast) * delta / 2.0;
        up += (vup + oldvup) * delta / 2.0;

        ex_n += (e_north + old_e_north) * delta / 2.0;
        ex_e += (e_east + old_e_east) * delta / 2.0;
        ex_u += (e_up + old_e_up) * delta / 2.0;

        decoder->ensembles.pop();
    }
}

DVL::~DVL()
{
    dvl->close();
    delete(decoder);
    delete(dvl);
}

void DVL::AHRSsub(const bbauv_msgs::imu_dataConstPtr &imu)
{
    yaw = imu->orientation.z;
}

void DVL::collect(ros::Publisher *publisher, ros::Publisher *epub)
{
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = north;
    odom.pose.pose.position.y = east;
    odom.pose.pose.position.z = up;
    odom.twist.twist.linear.x = vnorth;
    odom.twist.twist.linear.y = veast;
    odom.twist.twist.linear.z = vup;

    nav_msgs::Odometry earth;
    earth.pose.pose.position.x = ex_n;
    earth.pose.pose.position.y = ex_e;
    earth.pose.pose.position.z = ex_u;

    publisher->publish(odom);
    epub->publish(earth);
}

void DVL::zero(dvl::dvlConfig &config, uint32_t level)
{
    if(config.zero_distance){
        ROS_INFO("[DVL] Zeroing earth odom: level: %d", level);
        ex_e = ex_n = ex_u = 0;
        config.zero_distance = false;
    }
}

void DVL::zero_relative(dvl::dvlConfig &config, uint32_t level)
{
    if(config.zero_relative_distance){
        ROS_INFO("[DVL] Zeroing relative DVL distance: level: %d", level);
        north = east = up = 0;
        config.zero_relative_distance = false;
    }
}
