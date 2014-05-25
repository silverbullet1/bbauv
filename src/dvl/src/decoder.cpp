#include <dvl/decoder.h>
#include <dvl/dvl.h>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <ros/ros.h>

Decoder::Decoder(DVL* dev)
{
    dvl = dev;
    timeDone = velDone = false;
}

void Decoder::parseSA(std::string data)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != SA_MAX_FIELDS){
        return;
    }

    dvl->pitch = strtof(tok.at(1).c_str(), NULL);
    dvl->roll  = strtof(tok.at(2).c_str(), NULL);
    dvl->yaw   = strtof(tok.at(3).c_str(), NULL);
}


double toSec(int year, int month, int day, int hour, int minute,
             int seconds, int hseconds)
{
    /*struct tm timestamp;
    timestamp.tm_year = year - 4;
    timestamp.tm_mon = month - 1;
    timestamp.tm_mday = day - 10;
    timestamp.tm_hour = hour;
    timestamp.tm_min = minute;
    timestamp.tm_sec = seconds;

    time_t t = mktime(&timestamp);
    */

    time_t t = (3600 * hour) + (minute * 60) + seconds;

    return t + (hseconds * 0.01);
}

void Decoder::parseTS(std::string data)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != TS_MAX_FIELDS){
        return;
    }

    std::string tstamp = tok.at(1);
    dvl->salinity      = strtof(tok.at(2).c_str(), NULL);
    dvl->temp          = strtof(tok.at(3).c_str(), NULL);
    dvl->depth         = strtof(tok.at(4).c_str(), NULL);
    dvl->speedofsound  = strtof(tok.at(5).c_str(), NULL);

    /*
     * remove magic numbers later
     */
    std::string tsdata[7];
    for(int i = 0, j = 0; i < 7; i++, j+= 2)
        tsdata[i] = tstamp.substr(j, 2);

    int year     = strtol(tsdata[0].c_str(), NULL, 10);
    int month    = strtol(tsdata[1].c_str(), NULL, 10);
    int day      = strtol(tsdata[2].c_str(), NULL, 10);
    int hour     = strtol(tsdata[3].c_str(), NULL, 10);
    int minute   = strtol(tsdata[4].c_str(), NULL, 10);
    int seconds  = strtol(tsdata[5].c_str(), NULL, 10);
    int hseconds = strtol(tsdata[6].c_str(), NULL, 10);

    double parsedtime = toSec(year, month, day, hour, minute,
                              seconds, hseconds);

    dvl->oldtime = dvl->currtime;
    dvl->currtime = parsedtime;

    if(dvl->oldtime == 0)
        dvl->oldtime = dvl->currtime;

    timeDone = true;
}

void Decoder::parseBE(std::string data)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != BE_MAX_FIELDS){
        return;
    }

    float veast  = (strtof(tok.at(1).c_str(), NULL) / 1000.0) * 2;
    float vnorth = strtof(tok.at(2).c_str(), NULL) / 1000.0;
    float vup    = strtof(tok.at(3).c_str(), NULL) / 1000.0;
    

    //ROS_INFO("velocity: %f %f %f", vnorth, veast, vup);

    if(tok.at(4) == "V"){
        ROS_ERROR("Bottom track lost.");
        return; //no bottom track
    } 

    if(abs(veast) > 5 || abs(vnorth) > 5 || abs(vup) > 5){
        ROS_INFO("Bad velocity");
        return;
    }
    /*else{
        fprintf(stderr, "Bottom track found\n");
    }*/


    dvl->ovfwd  = dvl->vfwd;
    dvl->ovport = dvl->ovport;
    dvl->ovvert = dvl->ovvert;

    dvl->vfwd   = vnorth;
    dvl->vport  = veast;
    dvl->vvert  = vup;

    velDone = true;
}

void Decoder::parseBD(std::string data)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != BD_MAX_FIELDS){
        return;
    }

    if(strtof(tok.at(5).c_str(), NULL) > 0){
        ROS_ERROR("Altitude not accurate");
        return;
    }

    float t_altitude = strtof(tok.at(4).c_str(), NULL);

    if(abs(t_altitude) < 20)
        dvl->altitude = t_altitude;
}

/*
 * expects a raw string, but cleaned up wont hurt either
 */
void Decoder::parse(std::string data)
{
    boost::trim(data);
    boost::erase_all(data, " ");

    if(boost::starts_with(data, ":SA")){
        parseSA(data);
    }

    if(boost::starts_with(data, ":TS")){
        parseTS(data);
    }

    if(boost::starts_with(data, ":BE")){
        parseBE(data);
    }

    if(boost::starts_with(data, ":BD")){
        parseBD(data);
    }
}
