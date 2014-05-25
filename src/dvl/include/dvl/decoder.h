#ifndef DECODER_H
#define DECODER_H

#include <dvl/dvl.h>

#ifndef PD6_CONSTS
#define PD6_CONSTS

#define SA_MAX_FIELDS 4
#define TS_MAX_FIELDS 7
#define BE_MAX_FIELDS 5
#define BD_MAX_FIELDS 6

#endif

class DVL;

class Decoder{
    DVL *dvl;
    public:
        Decoder(DVL* dev);
        ~Decoder(){};

        bool timeDone, velDone;

        void parseBD(std::string data);
        void parseBE(std::string data);
        void parseTS(std::string data);
        void parseSA(std::string data);
        void parse(std::string data);
};

#endif /* DECODER_H */
