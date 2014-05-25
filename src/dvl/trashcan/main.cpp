/*
 * Entry point for the ExplorerDVL driver
 */

#include <ros/ros.h>
#include "../include/explorer.h"
#include <bbauv_msgs/imu_data.h>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>

float im_atof(std::string t)
{
    int m = 1;
    if(t[0] == '-'){
        m = -1;
    }

    boost::erase_all(t, "+");
    boost::erase_all(t, "-");
    float tmp = atof(t.c_str());
    return tmp * m;
}

void parseSA(std::string dat, bbauv_msgs::imu_data* imumsg)
{
    float pitch, roll, heading;
    std::vector<std::string> tok;
    boost::split(tok, dat, boost::is_any_of(","));
    
    if(tok.size() != 4)
        return;
    
    pitch   = im_atof(tok.at(1));
    roll    = im_atof(tok.at(2));
    heading = atof(tok.at(3).c_str());

    imumsg->orientation.x = roll;
    imumsg->orientation.y = pitch;
    imumsg->orientation.z = heading;
}

void parseBE(std::string dat)
{
    float east;
    float north;
    float vertical;
    bool status = false;

    std::vector<std::string> tok;
    boost::split(tok, dat, boost::is_any_of(","));
    if(tok.size() != 5)
        return;

    east = im_atof(tok.at(1));
    north = im_atof(tok.at(2));
    vertical = im_atof(tok.at(3));

    if(tok.at(4) == "A")
        status = true;
    else{
        ROS_ERROR("Bottom track lost");
        return;
    }

    if(abs(east) > 5 || abs(north) > 5){
        ROS_ERROR("Max velocity exceeded");
        return;
    }

    east /= 1000.0;
    north /= 1000.0;
    vertical /= 1000.0;

    ROS_INFO("e:%f,n:%f,v:%f", east, north, vertical);
}

/*
 * returns seconds
 */
double parseTime(std::string dat)
{
    char year[3], month[3], day[3], hour[3], minute[3], second[3],
        csecond[3];
    const char* src = dat.c_str();
    strncpy(year, src, 2);
    strncpy(month, src + 2, 2);
    strncpy(day, src + 4, 2);
    strncpy(hour, src + 6, 2);
    strncpy(minute, src + 8, 2);
    strncpy(second, src + 10, 2);
    strncpy(csecond, src+12, 2);

    year[2]    = '\0';
    month[2]   = '\0';
    day[2]     = '\0';
    hour[2]    = '\0';
    minute[2]  = '\0';
    second[2]  = '\0';
    csecond[2] = '\0';
}

void parseTS(std::string dat)
{
    double timenow;
    std::vector<std::string> tok;
    boost::split(tok, dat, boost::is_any_of(","));
    
    timenow = atof(tok.at(1).c_str());
    ltime = time;
    time = timenow;

   if(time - ltime > 5){
        ROS_ERROR("Time to last bottom track timed out");
   }
   
}

void parseBD(std::string dat)
{
    float altitude;
    float d_east, d_north, d_vert;

    std::vector<std::string> tok;
    boost::split(tok, dat, boost::is_any_of(","));

    if(tok.size() != 6)
        return;

    d_east = atof(tok.at(1).c_str());
    d_north = atof(tok.at(2).c_str());
    d_vert = atof(tok.at(3).c_str());
    float time = atof(tok.at(5).c_str());

    altitude = atof(tok.at(4).c_str());
    ROS_INFO("Distance: east: %fm, north: %fm, vert: %fm, TTL:%f",
             d_east, d_north, d_vert, time);
    ROS_INFO("Altitude: %f", altitude);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DVL");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(20);

    DVL dev("/dev/ttyDVL", 9600, 1500,
            "/dev/ttyDVLSensor", 9600, 1500);

    dev.setup();

    ros::Publisher dvl_imupub = nh.advertise<bbauv_msgs::imu_data>
        ("/DVL_imu", 1);

    bbauv_msgs::imu_data dvl_imu;
    std::string data;

    while(ros::ok()){

        data = dev.read();
        if(boost::starts_with(data, ":SA")){
            parseSA(data, &dvl_imu);
            dvl_imupub.publish(dvl_imu);
        }
        if(boost::starts_with(data, ":BE")){
            parseBE(data);
        }
        if(boost::starts_with(data, ":BD")){
            parseBD(data);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    /*
     * bbauv_msgs::imu_data test;
     * test.orientation.x = 0.12;
     * test.orientation.y = 1.21;
     * test.orientation.z = 254.32;
     * dev.populatesensors(&test);
     */
    return 0;
}
