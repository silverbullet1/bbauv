/*----------------decoder.h-------------------------------------*/
/*-------------------------------NOTICE-------------------------*/
/*---frame for both position and velocity is odom---------------*/
/*---x coordinate is EAST direction-----------------------------*/
/*---y coordinate is NORTH direction----------------------------*/
/*---z coordinate is vertically up------------------------------*/
/*--------------------------------------------------------------*/

#ifndef DECODER_H
#define DECODER_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <string>
#include <sstream>
#include <cerrno>
#include <climits>

//all the below define are from MSB to LSB
//fixed
#define HeaderID "7F7F"
#define FLdrSelected "0000"
#define VLdrSelected "0080"

//if water profiling is enabled
#define VelSelected "0100" //velocity
#define CorSelected "0200" //correlation magnitude
#define AmpSelected "0300" //echo intensity
#define PctSelected "0400" //percent good
//#define SttSelected 0x0500
#define BotSelected "0600"

using namespace std;

class Decoder
{
    public:
        typedef unsigned char uchar;
        typedef unsigned short ushort;
        typedef unsigned long ulong;

        typedef struct
        {
            int Min, Sec, Sec100;
        } TimeType;

        typedef struct
        {
            int Century,Year, Month, Day, Hour, Min, Sec, Sec100;
        } DateTimeType;

        typedef struct
        {
            int Version, Revision;
        } VersionType;

        typedef struct
        {
            int ChecksumOffset;
            int NDataTypes;
            int Offset[8];
        } HeaderType;

        typedef struct
        {
            VersionType CPUFirmware;
            int Configuration;
            int DummyDataFlag, Lag, NBeams, NBins;
            int PingsPerEnsemble, BinLength, BlankAfterTransmit;
            int SignalProMode, PctCorrelationLow, NCodeRepetitions, PctGoodMin;
            int ErrVelocityThres;
            TimeType TimeBetweenPings;
            int CoordSystemParms;
            int HeadingAlignment, HeadingBias;
            int SensorSource, AvailableSensors;
            int Bin1Dist, PulseLength;
            int StartDepthCell, EndDepthCell;
            int FalseTargetThres;
            int LagDist;
            string CPUSerial, SystemBandwidth;
            int BaseFreqIndex;
        } FixLeaderType;

        typedef struct
        {
            int EnsembleNumber;
            DateTimeType TSRecordingTime;
            int EnsembleNumberMSB;
            int BITResult, SpeedOfSound, Depth, Heading;
            int Pitch, Roll;
            int Salinity;
            int Temperature;
            TimeType MPT; //minimum pre-ping wait time between ping group in the ensemble
            int HeadingStddev, PitchStddev, RollStddev;
            int ADC[8]; //analog to digital channel 0 to 7
            int ErrorStt[4]; //error status long word
            int Pressure, PressureVar;
            DateTimeType TTRecordingTime;
        } VarLeaderType;

        typedef struct
        {
            int PingsPerEnsemble, EnsembleWait;
            int CorrelationMin, AmplitudeMin, PctGoodMin, BTMode;
            int ErrVelocityMax, Range[4];
            int Velocity[4];
            //int NSearchPings, NTrackPings;
            int Correlation[4], Amplitude[4], PctGood[4];
            int WaterLayerMin, WaterLayerNear, WaterLayerFar;
            int WVelocity[4];
            int WCorrelation [4], WAmplitude[4], WPctGood[4];
            int MaxTrackingDepth;
            int RSSIAmp[4]; //Receiver Signal Strength Indicator value
            int Gain;
            int RangeMSB [4];
        } BottomTrackType;

        Decoder();
        ~Decoder();

        //accept the hex string and a 2 bytes hex checksum
        //return matched (0) or error (1)
        bool check_checksum(string testStr,string checksum);

        //input: hex number from LSB to MSB (inversed)
        //output: the corresponding dec number
        int invByteHexToDec(string invHexNum);

        int inv2ndByteHexToDec(string invHexNum);

        //check the received ensemble (check Header ID and checksum)
        //return 0 if ok, 1 if error happened
        bool checkRcvEnsemble();

        //the following 4 functions receive the offset, then fill in 4 global var
        void solveHeader();
        void solveFixLeader(ushort offset);
        void solveVarLeader(ushort offset);
        void solveBotTrack(ushort offset);

        //print all global variables
        void printResult();

        //decode the ensemble
        //results are saved in to global variables
        void DecodeBBensemble(void);

        //setters
        void setRcvStr(string str);

        //getters
        string getPrintStr();
        HeaderType getHeader();
        FixLeaderType getFixLeader();
        VarLeaderType getVarLeader();
        BottomTrackType getBotTrack();

    private:
        /*--------------Global Variables--------------------*/
        HeaderType header;
        FixLeaderType fixLeader;
        VarLeaderType varLeader;
        BottomTrackType botTrack;

        string rcvStr;
        string printStr;
};
#endif
