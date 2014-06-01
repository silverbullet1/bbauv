#include <boost/algorithm/string.hpp>
#include "../include/decoder.h"

#define SA_MAX_FIELDS 4
#define TS_MAX_FIELDS 7
#define BE_MAX_FIELDS 5

PD6Decoder::PD6Decoder(DVL* dev){
    dvl = dev;
    buffer = "";
}

void parseSA(std::string data, ensemble *en)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != SA_MAX_FIELDS){
        return;
    }

    en->pitch = strtof(tok.at(1).c_str(), NULL);
    en->roll  = strtof(tok.at(2).c_str(), NULL);
    en->yaw   = strtof(tok.at(3).c_str(), NULL);
}

void parseBE(std::string data, ensemble *en)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != BE_MAX_FIELDS){
        return;
    }

    float veast  = (strtof(tok.at(1).c_str(), NULL) / 1000.0);
    float vnorth = strtof(tok.at(2).c_str(), NULL) / 1000.0;
    float vup    = strtof(tok.at(3).c_str(), NULL) / 1000.0;

    if(tok.at(4)[0] == 'V'){
        ROS_ERROR("Bottom track lost.");
        en->status = false;
        return; //no bottom track
    }

    en->bv_east = veast;
    en->bv_north = vnorth;
    en->bv_up = vup;
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

void parseTS(std::string data, ensemble *en)
{
    std::vector<std::string> tok;
    boost::split(tok, data, boost::is_any_of(","));

    if(tok.size() != TS_MAX_FIELDS){
        return;
    }

    std::string tstamp = tok.at(1);
    en->salinity       = strtof(tok.at(2).c_str(), NULL);
    en->temperature    = strtof(tok.at(3).c_str(), NULL);
    en->depth          = strtof(tok.at(4).c_str(), NULL);
    en->speed_of_sound = strtof(tok.at(5).c_str(), NULL);

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

    en->timestamp = parsedtime;
}


void PD6Decoder::parse()
{
    size_t first, last;
    first = buffer.find(":SA");
    if(first == buffer.npos)
        return;
    last = buffer.rfind(":SA");
    if(last == first)
        return;

    std::string to_parse = buffer.substr(first, last - first);
    buffer = buffer.substr(last, last - first);

    //ROS_INFO("Parsed: %s", to_parse.c_str());
    std::vector<std::string> tok;
    boost::split(tok, to_parse, boost::is_any_of("\n"));
    ensemble en;
    for(int i = 0; i < (int) tok.size(); i++){
        if(boost::starts_with(tok.at(i), ":SA")){
            en.status = true;
            parseSA(tok.at(i), &en);
            parseTS(tok.at(i + 1), &en);
            parseBE(tok.at(i + 4), &en);
            //parseBD(tok.at(i + 5), &en);
            ensembles.push(en);
            //ROS_INFO("pushing: %lf", en.timestamp);
            i = i + 5;
        }
    }
}
