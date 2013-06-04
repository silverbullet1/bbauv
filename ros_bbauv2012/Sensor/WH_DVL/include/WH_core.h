/*-------------------WH_core.h--------------------------------------
-----frame for both position and velocity is odom-------------------
-----x coordinate is EAST direction---------------------------------
-----y coordinate is NORTH direction--------------------------------
-----z coordinate is vertically up----------------------------------
-----units are meter, second, degree--------------------------------
--------------------------------------------------------------------*/

#ifndef WH_CORE_H
#define WH_CORE_H

// System includes.
#include <math.h>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ROS includes.
#include <ros/ros.h>

// Local includes.
#include "decoder.h"
#include "serial.h"
#include "timing.h"

using namespace std;

/******************************
 *
 * #defines
 *
 *****************************/

/* Command Set Summary for the Workhorse DVL. */
#ifndef WH_COMMANDS
#define WH_COMMANDS
/** @name Command values for the DVL. */
//@{
#define CMD_MAX_LENGTH         40
#define CMD_BREAK              "+++"
#define CMD_BIN_OUTPUT         "CF11110"
#define CMD_HEX_OUTPUT         "CF11210"
#define CMD_TIME_PER_ENB       "TE"
#define CMD_TIME_BETWEEN_PINGS "TP"
#define CMD_SET_TS_RTC         "TS"
#define CMD_SLEEP              "CL"
#define CMD_DEFAULT_SETTINGS   "CR1"
#define CMD_BOTTOM_MODE        "BM5"
#define CMD_BOT_PINGS          "BP1"
#define CMD_MAX_TRACK_DEPTH    "BX540"
#define CMD_HEADING_ALIGN      "EA+04500"
#define CMD_HEADING_BIAS       "EB0"
#define CMD_TRANSDUCER_DEPTH   "ED10"
#define CMD_SALINITY           "ES0"
#define CMD_COORDINATE         "EX11111"
#define CMD_SENSOR_SOURCE      "EZ1111101" 
#define CMD_VERIFY_COMPASS     "AX"
#define CMD_STOP_VERIFYING     "Q"
#define CMD_CALIBRATE_COMPASS  "AF"
#define CMD_SAVE_SETTINGS	   "CK"
#define CMD_START_PINGING      "CS"
#define CMD_END                "\r\n"
//@}
#endif // WH_COMMANDS

#ifndef WH_HEADER
#define WH_HEADER_ID "7F7F"
#endif // WH_HEADER

#ifndef WH_STRING_SIZE
#define WH_STRING_SIZE 8192
#endif // WH_STRING_SIZE

#ifndef WH_DELAY
#define WH_SERIAL_DELAY 200000
#define WH_CMD_DELAY 3000000
#define WH_DIST_TIMEOUT 5
#endif // WH_SERIAL_DELAY

#ifndef WH_ERROR_CONDITION
#define WH_LOST_BOTTOM -32.768
#endif // WH_ERRROR_CONDITION

#ifndef WH_ERROR_HEADER
#define WH_SUCCESS	        1
#define WH_ERROR_HEADER    -1
#define WH_ERROR_CHECKSUM  -2
#define WH_ERROR_LENGTH    -3
#define WH_ERROR_VALID_MSG -4
#endif // WH_ERROR_HEADER

/******************************
 *
 * Classes
 *
 *****************************/

class DVL : public Serial
{
public:
    //! Constructor.
    //! \param _portname The name of the port that the compass is connected to.
    //! \param _baud The baud rate that the DVL is configured to communicate at.
    //! \param _initTime The amount of time to try establishing communications with the DVL before timing out.
    DVL(string _portname, int _baud, int _initTime);

    //! Destructor.
    ~DVL();

    //! Establish communications with the DVL.
    void setup();

    //! Getting the raw data from the serial port, save to printStr
    // to be printed directly to the screen through WH_ros
    void getRawData();

    //! Get data from DVL.
    void getData();

    //! Looks for valid data from the compass for a specified amount of time.
    void init();

    //! Zero all the travelled distances
    void zeroDistance();

    //! send a command string to the DVL
    void sendCommand(string command, bool send_break);

    //! send break command
    void sendBreak();

    //! change output to hex
    void hexOutput();

    //! change output to binary
    void binOutput();

    //! set time per ensemble
    void timePerEnsemble(double seconds);

    //! set time betweenPings
    void timeBetweenPings(double seconds);

    //! synchronize the TS RTC time with the computer time
    void setTSClock();

    //! set whether sleep between pings or not
    void sleepBetweenPings(bool sleep);

    //! set the current settings as default settings
    void saveSettings();

    //! reset all settings by sending the factory default settings
    // command and all the basci settings commands
    void resetSettings();

    //! start pinging
    void startPinging();

    // verify the internal compass
    void verifyCompass();
    void stopVerifying();

    // start the calibration process
    void calibrateCompass();
    void stopCalibrating();
   
    //! return the number of seconds from 1970 to the input time
    double toSec(int year, int mon, int day, int hour, int min, int sec, int sec100);

    // get the string to be printed out
    string getPrintString();

    // get the string of data from decoder
    string getDataString();

    //! The file descriptor used to communicate with the compass.
    int fd;

    //! The amount of time attempting to set up communications with the compass.
    int init_time;

    //! Pointer to a timer.
    Timing *timer;


protected:
    /*----------------DATA NEEDED FOR ROS NODE---------------------*/
    //coordinate frame
    string header_frame_id;
    string child_frame_id;

    //time
    int year, month, day, hour, min, sec, sec100;
    double totalSec; //total seconds from 1970, including sec100

    //navigation data
    double x,y,z;
    double angX, angY, angZ;
    double posCov[36];
    double xvel,yvel,zvel;
    double angXvel,angYvel,angZvel;
    double velCov[36];

    //data needed for integrating and diffentiating
    double lastAngX, lastAngY, lastAngZ;
    double lastXvel, lastYvel, lastZvel;
    double lastTotalSec;

    //data needed to compute covariance matrix
    bool z_vel_var_set;
    int count;
    double start_time;
    double xy_vel_error;
    double xy_vel_var, z_vel_var;
    double ang_x_vel_var, ang_y_vel_var, ang_z_vel_var; 
    double ang_x_var, ang_y_var, ang_z_var;
    double last_ang_x_var, last_ang_y_var, last_ang_z_var;
    double mean_xy_vel;
    double square_mean_xy_vel;


private:
    //! The full received string
    string rcvStr;

    //! The receive string to be decoded
    string dataStr;

    //the decoder object for decoding data
    Decoder decoder; 

    //! Decoder data
    Decoder::VarLeaderType varLeader;
    Decoder::BottomTrackType botTrack;

    //! Searches a buffer looking for the start and end sequences.
    void findData();

    //! Parses a message for compass data.
    void decodeData();

    //! assign data name from decoder to ROS
    void assignData();

    //! integrate velocity data to get the position
    void computeDistance();

    //! differentiate to get angular velocity
    void computeAngVel();

    //! Whether the compass was initialized correctly.
    bool DVL_initialized;

    //! Whether a complete message was found after reading compass data.
    bool found_complete_message;

    //! string to be passed to WH_ros when neccessary
    string printStr;
};

#endif // WH_CORE_H
