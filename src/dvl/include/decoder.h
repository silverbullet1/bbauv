#ifndef DECODER_H
#define DECODER_H
#include <string>
#include <queue>
#include "DVL.h"

class DVL;

struct ensemble{
    float roll;
    float pitch;
    float yaw;

    double timestamp;
    float temperature;
    float speed_of_sound;
    uint8_t test;

    float bv_east;
    float bv_north;
    float bv_up;
    bool status;

    float wv_east;
    float wv_north;
    float wv_up;

    float xeast;
    float xnorth;
    float xup;

    float altitude;
    float depth;
    float salinity;
    double ttl;
};

class PD6Decoder{
    DVL *dvl;
    public:
        std::queue<ensemble> ensembles;
        std::string buffer;
        PD6Decoder(DVL*);
        ~PD6Decoder(){}
        void parse();
};

#endif /* DECODER_H */
