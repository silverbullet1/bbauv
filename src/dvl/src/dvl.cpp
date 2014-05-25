#include <dvl/dvl.h>
#include <dvl/decoder.h>
#include <fstream>
#include <numeric>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

using std::string;

DVL::DVL(string dport, int dbaud, int dtimeout,
         string sport, int sbaud, int stimeout)
{
    dvl_port           = dport;
    dvl_baud           = dbaud;
    dvl_timeout        = dtimeout;
    dvl_sensor_port    = sport;
    dvl_sensor_baud    = sbaud;
    dvl_sensor_timeout = stimeout;

    decoder            = new Decoder(this);

    vfwd = vport = vvert = ovfwd = ovport = ovvert = 0;
}

DVL::~DVL()
{

    dvl->flush();
    dvl->close();
    //dvl_sensor->flush();
    //dvl_sensor->close();

    delete(dvl);
    //delete(dvl_sensor);
}

void DVL::sendBreak()
{
    std::string br("+++\r\n");
    dvl->flushInput();
    dvl->flushOutput();
    dvl->write(br);
    std::vector<std::string> lines;
    lines = dvl->readlines();
    /*
    for(int i = 0; i < (int) lines.size(); i++){
        boost::trim(lines.at(i));
        fprintf(stdout, "%s\n", lines.at(i).c_str());
    }
    */
    dvl->write("CS\r\n");
}

bool DVL::setup()
{
    dvl = new serial::Serial(dvl_port, dvl_baud,
                             serial::Timeout::simpleTimeout(dvl_timeout));
    //dvl_sensor = new serial::Serial(dvl_sensor_port, dvl_sensor_baud,
    //                                serial::Timeout::simpleTimeout(
    //                                        dvl_sensor_timeout
    //                                    ));

    if(!dvl->isOpen()){
        return false;
    }
    dvl->flush();

    sendBreak();

    vfwd = vport = vvert = ovfwd =
        ovvert = ovport = 0;
    x = y = z = 0;

    return true;
}

void DVL::integrate()
{
    if(abs(vfwd) > 5 || abs(vport) > 5 || abs(vvert) > 5 ||
       abs(ovfwd) > 5 || abs(ovport) > 5 || abs(ovvert) > 5){
        return;
    }

    double delta = currtime - oldtime;

    //ROS_INFO("Integrating: %f %f %f %lf", y, vport, ovport, delta);
    
    x += (vfwd + ovfwd) * delta / 2.0;
    y += (vport + ovport) * delta / 2.0;
    z += (vvert + ovvert) * delta / 2.0;

    decoder->timeDone = decoder->velDone = false;

    fprintf(stdout, "%f,%f,%f\n", x, y, z);
    decoder->timeDone = false;
    decoder->velDone = false;
}

/*
 * Local test using a minicom capture file
 */
void DVL::test(std::string filename)
{
    std::ifstream capture(filename);
    for(std::string line; getline(capture, line);){
        decoder->parse(line);
        integrate();
    }
}

/*
 * ros calls this
 */
void DVL::runOnce()
{

    std::string data;
    std::vector<std::string> ensemble;
    data = dvl->readline();
    /*data = dvl->readline();
    boost::trim(data);
    //decoder->parse(data);
    ROS_INFO("%s", data.c_str());
    //integrate();
    */
    if(boost::starts_with(data, ":SA")){
        ensemble = dvl->readlines(200, std::string("\r\n"));
        //for(int i = 0; i < (int) ensemble.size(); i++)
        //    ROS_INFO("%s", ensemble.at(i).c_str());
        //ROS_INFO("ENSEMBLE END");
    } else{
        return runOnce();
    }
    
    for(int i = 0; i < (int) ensemble.size(); i++){
        decoder->parse(ensemble.at(i));
    }

    integrate();
}

uint8_t NMEAchecksum(std::string n)
{
    return accumulate(n.begin(), n.end(), 1, [](uint8_t c, char d){
                        if(d != '$') return c ^ d;
                        else return 1;
                      });
}

float radtodeg(float rad)
{
    return rad * (180 / M_PI);
}

void DVL::populatesensors(const bbauv_msgs::imu_data::ConstPtr &imu)
{
    float roll, pitch, yaw;
    roll  = radtodeg(imu->orientation.x);
    pitch = radtodeg(imu->orientation.y);
    yaw   = radtodeg(imu->orientation.z);

    char nmea[] = "$PRDID,%0.2f,%0.2f,%0.2f,*";
    char buff[256];
    snprintf(buff, 256, nmea, pitch, roll, yaw);
    uint8_t checksum = NMEAchecksum(buff);
    strcat(buff, "%02X\r\n");
    snprintf(buff, 256, buff, checksum);
    //fprintf(stdout, "%s\n", buff);
    //dvl_sensor->write(buff);
}

void DVL::fillpose(nav_msgs::Odometry* odom)
{
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.position.z = z;

    odom->twist.twist.linear.x = vfwd;
    odom->twist.twist.linear.y = vport;
    odom->twist.twist.linear.z = vvert;
}

void DVL::fillalt(std_msgs::Float32 *alt)
{
    alt->data = altitude;
}
