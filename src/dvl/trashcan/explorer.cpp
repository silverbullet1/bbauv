#include "../include/explorer.h"
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <numeric>

//#pragma GCC diagnostic ignored "-Wwrite-strings"

DVL::DVL(std::string p, int b, int t,
         std::string sensorport, int sensorbaud, int sensortimeout)
{
    port = p;
    baudrate = b;
    timeout = t;

    dev = new serial::Serial(port, baudrate,
                             serial::Timeout::simpleTimeout(timeout));

    //sensors = new serial::Serial(sensorport, sensorbaud,
    //                        serial::Timeout::simpleTimeout(sensortimeout));

}

void DVL::setup()
{
    if(!dev->isOpen()){
        fprintf(stderr, "Device is not open.");
        exit(1);
    }
    if(!dev->available())
        sendBreak();

    /*if(!sensors->isOpen()){
        fprintf(stderr, "No sensor port available to stream data");
        exit(1);
    }*/
}

void DVL::sendBreak()
{
    std::string br("+++\r\n");
    dev->flushInput();
    dev->flushOutput();
    dev->write(br);
    std::vector<std::string> lines;
    lines = dev->readlines();
    /*
    for(int i = 0; i < (int) lines.size(); i++){
        boost::trim(lines.at(i));
        fprintf(stdout, "%s\n", lines.at(i).c_str());
    }
    */
    dev->write("CS\r\n");
}

uint8_t NMEAchecksum(std::string n)
{
    return accumulate(n.begin(), n.end(), 1, [](uint8_t c, char d){
                        if(d != '$') return c ^ d;
                        else return 1;
                      });
}

std::string DVL::read()
{
    std::string data;
    data = dev->readline();
    boost::trim(data);
    boost::erase_all(data, " ");
    return data;
}

void DVL::populatesensors(bbauv_msgs::imu_data *imu)
{
    float roll, pitch, yaw;
    roll  = imu->orientation.x;
    pitch = imu->orientation.y;
    yaw   = imu->orientation.z;

    char nmea[] = "$PRDID,%0.2f,%0.2f,%0.2f,*";
    char buff[256];
    snprintf(buff, 256, nmea, pitch, roll, yaw);
    uint8_t checksum = NMEAchecksum(buff);
    strcat(buff, "%02X\r\n");
    snprintf(buff, 256, buff, checksum);
    //sensors->write(buff);
}
