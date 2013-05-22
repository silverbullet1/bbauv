//WH_core.cpp

#include "WH_core.h"

/*------------------------------------------------------------------------------
 * WH()
 * Constructor.
 *----------------------------------------------------------------------------*/

DVL::DVL(string _portname, int _baud, int _init_time) : Serial::Serial(_portname, _baud) {
    // Initialize variables.
    baud      = _baud;
    init_time = _init_time;
    portname  = _portname;

    // Start a new timer.
    timer = new Timing(init_time);

    decoder = Decoder();

    z_vel_var_set = false;

    // Set up the DVL.
    setup();

    // Zero all the distance
    x = 0;
    y = 0;
    z = 0;

    // zero time data
    totalSec   = 0;
    count      = 0;
    start_time = 0;

    // zero mean data
    xy_vel_var         = 0;
    z_vel_var          = 0;
    ang_x_var          = 0;
    ang_y_var          = 0;
    ang_z_var          = 0;
    last_ang_x_var     = 0;
    last_ang_y_var     = 0;
    last_ang_z_var     = 0;
    mean_xy_vel        = 0;
    square_mean_xy_vel = 0;

    // zero the covariance matrix
    for (int i=0; i < 36; i++) {
        posCov[i] = 0;
        velCov[i] = 0;
    }

    // clean all strings
    rcvStr   = "";
    dataStr  = "";
    printStr = "";
} // end DVL()


/*------------------------------------------------------------------------------
 * ~DVL()
 * Destructor.
 *----------------------------------------------------------------------------*/

DVL::~DVL() {
    // Close the open file descriptors.
    if (fd > 0) {
        close(fd);
    }
    if (Serial::fd > 0) {
        close(Serial::fd);
    }
    delete timer;
} // end ~DVL()


/*------------------------------------------------------------------------------
 * void setup()
 * Initializes communications with the workhorse DVL. Sets up a file
 * descriptor for further communications.
 *----------------------------------------------------------------------------*/

void DVL::setup() {
    // Declare variables.
    DVL_initialized = false;

    // Check if the DVL serial port was opened correctly.
    if (Serial::fd > 0) {
        // Initialize the timer.
        timer->set();

        // Check for valid DVL data until the timer expires or valid data is found.
        while (!timer->checkExpired()) {
            // Try to initialize the DVL to make sure that we are actually getting valid data from it.
            init();
            if (DVL_initialized) {
                break;
            }
        }
    }
    else {
        cout << "Serial port not opened" << endl;
    }

    // Set the file descriptor.
    fd = Serial::fd;
} // end setup()

/*------------------------------------------------------------------------------
 * void init()
 * Looks for valid data from the compass.
 *----------------------------------------------------------------------------*/

void DVL::init() {
    // Initialize variables.
    DVL_initialized = true;

    // Get the number of bytes available on the serial port.
    tcflush(Serial::fd, TCIFLUSH);
    usleep(WH_SERIAL_DELAY);
} // end init()

/*
 * Getting raw data directly from the serial port
 * save data to printStr to be printed directly to the screen
 * through WH_ros
 */
void DVL::getRawData() {
    // Declare variables.
    int bytes_to_discard = 0;

    // Get the number of bytes available on the serial port.
    getBytesAvailable();

    // Make sure we don't read too many bytes and overrun the buffer.
    if (bytes_available >= SERIAL_MAX_DATA) {
        bytes_available = SERIAL_MAX_DATA - 1;
    }
    if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA) {
        bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
        memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
    }

    // Read the data off the serial port
    if (bytes_available > 0) {
        recv();
        string newStr(Serial::buf_recv);
        printStr += newStr;
    }
}

/*------------------------------------------------------------------------------
 * void getData()
 * Get data from DVL.
 *----------------------------------------------------------------------------*/

void DVL::getData() {
    // Declare variables.
    int bytes_to_discard = 0;
    found_complete_message = false;

    // Get the number of bytes available on the serial port.
    getBytesAvailable();

    // Make sure we don't read too many bytes and overrun the buffer.
    if (bytes_available >= SERIAL_MAX_DATA) {
        bytes_available = SERIAL_MAX_DATA - 1;
    }
    
    if (bytes_available + strlen(buf_recv) >= SERIAL_MAX_DATA) {
        bytes_to_discard = bytes_available + strlen(buf_recv) - SERIAL_MAX_DATA - 1;
        memmove(buf_recv, &buf_recv[bytes_to_discard], bytes_to_discard);
    }

    //memset(&buf_recv, 0, sizeof(buf_recv));

    // Read data off the serial port.
    //cout << "bytes_available: " << bytes_available << endl;
    if (bytes_available > 0) {
        recv();
        // Look for entire message.
        findData();
        if (found_complete_message) {
            // Decode data.
            decodeData();
            assignData();
            computeDistance();
            computeAngVel();
        }
    }
} // end getData()

/*------------------------------------------------------------------------------
 * void findData()
 * Searches a buffer looking for the start and end sequences.
 *----------------------------------------------------------------------------*/

void DVL::findData() {
    //cout << "DVL::findData() is called" << endl;
    int start_location, end_location;
    found_complete_message = false;

    string newStr(Serial::buf_recv);
    rcvStr += newStr;

    //cout << "Serial::buf_recv length: " << strlen(Serial::buf_recv) << endl;
    //cout << "newStr: " << newStr << endl;
    //cout << "rcvStr: " << rcvStr << endl;
    /*
    int i = 0;
    while (rcvStr[i] != '\0') {
        //cout << "inside while loop is called" << endl;
        switch (rcvStr[i]) {
            case '\n':
                printf("\\n");
                break;
            case '\r':
                printf("\\r");
                break;
            case '\t':
                printf("\\t");
                break;
            default:
                putchar(rcvStr[i]);
                break;
        }
        i++;
    }*/

    start_location = 0;
    end_location = 0;
    start_location = rcvStr.find(WH_HEADER_ID);
    if (start_location == (int)string::npos) {
        cout << "WH_core cannot find header ID" << endl;
        return;
    }

    end_location = rcvStr.find(CMD_END, start_location);
    if (end_location == (int)string::npos) {
        cout << "WH_core cannot find newline character" << endl;
        return;
    }

    // Find the last data string
    int temp_start = start_location;
    int temp_end = end_location;
    while (temp_start != (int)string::npos && temp_end != (int)string::npos) {
        start_location = temp_start;
        end_location   = temp_end;
        temp_start     = rcvStr.find(WH_HEADER_ID, temp_end);
        temp_end       = rcvStr.find(CMD_END, temp_start);
    }


    dataStr = rcvStr.substr(start_location, end_location - start_location);
    /*
    i = 0;
    while (dataStr[i] != '\0') {
        switch (dataStr[i]) {
            case '\n':
                printf("\\n");
                break;
            case '\r':
                printf("\\r");
                break;
            case '\t':
                printf("\\t");
                break;
            default:
                putchar(dataStr[i]);
                break;
        }
        i++;
    }*/
    rcvStr.erase(0, end_location + 2);
    found_complete_message = true;

    return;
} // end findData()


/*------------------------------------------------------------------------------
 * void decodeData()
 * decode a message for DVL data.
 *----------------------------------------------------------------------------*/

void DVL::decodeData() {
    decoder.setRcvStr(dataStr);
    if (decoder.checkRcvEnsemble() != 0) {
        cout << "\nWH_core::invalid ensemble" << endl;
        return;
    }
    decoder.DecodeBBensemble();

    varLeader = decoder.getVarLeader();
    botTrack  = decoder.getBotTrack();

    return;
} // end decodeData()

void DVL::assignData() {
    header_frame_id = "odom"; 
    child_frame_id  = "base_footprint";

    year   = (varLeader.TSRecordingTime).Year;
    month  = (varLeader.TSRecordingTime).Month;
    day    = (varLeader.TSRecordingTime).Day;
    hour   = (varLeader.TSRecordingTime).Hour;
    min    = (varLeader.TSRecordingTime).Min;
    sec    = (varLeader.TSRecordingTime).Sec;
    sec100 = (varLeader.TSRecordingTime).Sec100;

    lastTotalSec = totalSec;
    totalSec     = toSec(year,month,day,hour,min,sec,sec100);

    lastAngX = angX;
    lastAngY = angY;
    lastAngZ = angZ;

    angX = varLeader.Roll    / 100.0;
    angX = angX * M_PI / 180.0;
    angY = varLeader.Pitch   / 100.0;
    angY = angY * M_PI / 180.0;
    angZ = varLeader.Heading / 100.0;
    angZ = angZ * M_PI / 180.0;

    lastXvel = xvel;
    lastYvel = yvel;
    lastZvel = zvel;

    xvel = -botTrack.Velocity[1] / 1000.0;//forward
    yvel = botTrack.Velocity[0] / 1000.0;//North
    zvel = -botTrack.Velocity[2] / 1000.0;//vertical

    botTrack.Velocity[3] = botTrack.Velocity[3] / 1000.0; // error

    return;
}

void DVL::computeDistance() {
    double delta = totalSec - lastTotalSec;
    if (xvel == WH_LOST_BOTTOM || lastXvel == WH_LOST_BOTTOM
        || yvel == WH_LOST_BOTTOM || lastYvel == WH_LOST_BOTTOM
        || zvel == WH_LOST_BOTTOM || lastZvel == WH_LOST_BOTTOM) {
        ROS_INFO("DVL::Bottom lock is lost");
        return;
    }
    else {
        x += (xvel + lastXvel) * delta / 2.0;
        y += (yvel + lastYvel) * delta / 2.0;
        z += (zvel + lastZvel) * delta / 2.0;
    }

    //data to fill in covariance matrix
    if (start_time == 0) {
        start_time = totalSec;
        ROS_INFO("start time: %lf", start_time);
    }
    
    xy_vel_error = botTrack.Velocity[3];

    mean_xy_vel = (mean_xy_vel * count + xy_vel_error) * 1.0 / (count + 1);
    
    square_mean_xy_vel = (square_mean_xy_vel*count + xy_vel_error*xy_vel_error);
    square_mean_xy_vel = square_mean_xy_vel * 1.0 / (count + 1);
    
    xy_vel_var = square_mean_xy_vel - mean_xy_vel * mean_xy_vel;

    // set z velocity variance equal to xy if no input through param
    if (z_vel_var_set == false) z_vel_var = xy_vel_var;

    // fill in covariance matrix
    posCov[0]  = xy_vel_var * (totalSec - start_time);
    posCov[7]  = xy_vel_var * (totalSec - start_time);
    posCov[14] = z_vel_var  * (totalSec - start_time);
    //cout << "z_vel_var_set: " << z_vel_var_set << endl;
    //cout << "z_vel_var    : " << z_vel_var << endl;
    //cout << "passed time  : " << totalSec - start_time << endl;


    velCov[0]  = xy_vel_var;
    velCov[7]  = xy_vel_var;
    velCov[14] = z_vel_var;

    count++;

    return;
}

void DVL::computeAngVel() {
    double delta = totalSec - lastTotalSec;
    angXvel = (angX - lastAngX) * 1.0 / delta;
    angYvel = (angY - lastAngY) * 1.0 / delta;
    angZvel = (angZ - lastAngZ) * 1.0 / delta;

    //data to fill in covariance matrix
    if (start_time == 0) start_time = totalSec;

    last_ang_x_var = ang_x_var;
    last_ang_y_var = ang_y_var;
    last_ang_z_var = ang_z_var;

    ang_x_var = varLeader.RollStddev * varLeader.RollStddev;
    ang_y_var = varLeader.PitchStddev * varLeader.PitchStddev;
    ang_z_var = varLeader.HeadingStddev * varLeader.HeadingStddev;

    if ( ang_x_var == 0) 
        ROS_INFO("Zero roll variance");
    if ( ang_y_var == 0)
        ROS_INFO("Zero pitch variance");
    if ( ang_z_var == 0)
        ROS_INFO("Zero heading variance");

    ang_x_vel_var = (ang_x_var + last_ang_x_var) * 1.0 / delta;
    ang_y_vel_var = (ang_y_var + last_ang_y_var) * 1.0 / delta;
    ang_z_vel_var = (ang_z_var + last_ang_z_var) * 1.0 / delta;

    // fill in the covariance matrix
    posCov[21] = ang_x_var * M_PI / 180.0;
    posCov[28] = ang_y_var * M_PI / 180.0;
    posCov[35] = ang_z_var * M_PI / 180.0;

    velCov[21] = ang_x_vel_var * M_PI / 180.0;
    velCov[28] = ang_y_vel_var * M_PI / 180.0;
    velCov[35] = ang_z_vel_var * M_PI / 180.0;

    return;
}

void DVL::zeroDistance() {
    // distance
    x = 0;
    y = 0;
    z = 0;

    // time variable
    count      = 0;
    start_time = 0;

    // linear variances
    xy_vel_var = 0;

    // angular variances
    ang_x_var = 0;
    ang_y_var = 0;
    ang_z_var = 0;
    last_ang_x_var = 0;
    last_ang_y_var = 0;
    last_ang_z_var = 0; 

    // linear means 
    mean_xy_vel = 0;
    square_mean_xy_vel = 0;

    return;
}

void DVL::sendCommand(string command, bool send_break) {
    // Declare variables.
    char cmd[CMD_MAX_LENGTH];

    int len;
    Serial::buf_send = cmd;

    // send command.
    if (Serial::fd > 0) {
        if (send_break) {
            // send out break
            sendBreak();
        }

        // send out command.
        len = command.length();
        for (int i=0; i < len; i++)
            cmd[i] = command[i];
        length_send = len;
        send();
        usleep(WH_SERIAL_DELAY);

        // send out the ending command.
        len = strlen(CMD_END);
        for (int i=0; i < len; i++)
            cmd[i] = CMD_END[i];
        length_send = len;
        send();
        usleep(WH_SERIAL_DELAY);
    }

    return;
}

void DVL::sendBreak() {
    /*
       char cmd[CMD_MAX_LENGTH];
       int len;
       Serial::buf_send = cmd;
    */
    // flush the all buffers
    flushInput();
    rcvStr   = "";
    dataStr  = "";
    printStr = "";

    /*
    if (Serial::fd > 0) {
    // send the BREAK
    len = strlen(CMD_BREAK);
    for (int i=0; i < len; i++)
    cmd[i] = CMD_BREAK[i];
    Serial::length_send = len;
    send();
    usleep(WH_SERIAL_DELAY);

    // send out the ending command.
    len = strlen(CMD_END);
    for (int i=0; i < len; i++)
    cmd[i] = CMD_END[i];
    length_send = len;
    send();
    usleep(WH_SERIAL_DELAY);

    // delay time for the DVL to change settings
    usleep(WH_CMD_DELAY);
    }*/
    sendHardBreak();
    // delay time for DVL to change settings
    usleep(WH_CMD_DELAY);
}

void DVL::hexOutput() {
    sendCommand(CMD_HEX_OUTPUT, true);
}

void DVL::binOutput() {
    sendCommand(CMD_BIN_OUTPUT, true);
}

void DVL::timePerEnsemble(double seconds) {
    char cmd[CMD_MAX_LENGTH];
    sprintf(cmd, "%s00:00:%05.2lf", CMD_TIME_PER_ENB, seconds);
    string cmdStr = cmd;
    sendCommand(cmdStr, true);
}

void DVL::timeBetweenPings(double seconds) {
    char cmd[CMD_MAX_LENGTH];
    sprintf(cmd, "%s00:%05.2lf", CMD_TIME_BETWEEN_PINGS, seconds);
    string cmdStr = cmd;
    sendCommand(cmdStr, true);
}

void DVL::setTSClock() {   
    time_t t = time(0);
    struct tm * now = localtime(&t);
    char cmd[CMD_MAX_LENGTH];

    //TSyy/mm/dd, hh:mm:ss
    sprintf(cmd, "%s%02d/%02d/%02d, %02d:%02d:%02d", CMD_SET_TS_RTC,
            now->tm_year - 100,
            now->tm_mon + 1,
            now->tm_mday,
            now->tm_hour,
            now->tm_min,
            now->tm_sec);
    string cmdStr = cmd;
    sendCommand(cmdStr, true);
}

void DVL::sleepBetweenPings(bool sleep) {
    char cmd[CMD_MAX_LENGTH];
    sprintf(cmd, "%s%d", CMD_SLEEP, sleep);
    string cmdStr = cmd;
    sendCommand(cmdStr, true);
}

void DVL::saveSettings() {
    sendCommand(CMD_SAVE_SETTINGS, true);
}

void DVL::resetSettings() {
    sendCommand(CMD_DEFAULT_SETTINGS, true);
    hexOutput();
    sendCommand(CMD_BOTTOM_MODE, true);
    sendCommand(CMD_BOT_PINGS, true);
    sendCommand(CMD_MAX_TRACK_DEPTH, true);
    sendCommand(CMD_HEADING_ALIGN, true);
    sendCommand(CMD_HEADING_BIAS, true);
    sendCommand(CMD_TRANSDUCER_DEPTH, true);
    sendCommand(CMD_SALINITY, true);
    sendCommand(CMD_COORDINATE, true);
    sendCommand(CMD_SENSOR_SOURCE, true);
    timePerEnsemble(0.1);
    timeBetweenPings(0.0);
    saveSettings();
}

void DVL::startPinging() {
    sendCommand(CMD_START_PINGING, true);
    flushInput();
    rcvStr   = "";
    dataStr  = "";
    printStr = "";
}


void DVL::verifyCompass() {
    sendCommand(CMD_VERIFY_COMPASS, true);
}

void DVL::stopVerifying() {
    sendCommand(CMD_STOP_VERIFYING, false);
}

void DVL::calibrateCompass() {
    sendCommand(CMD_CALIBRATE_COMPASS, true);
}

void DVL::stopCalibrating() {
    sendCommand(CMD_END, false);
}

double DVL::toSec(int year, int mon, int day, int hour, int min, int sec, int sec100) {
    struct tm givenTime;
    givenTime.tm_year = year + 2000 - 1900;
    givenTime.tm_mon  = mon - 1;
    givenTime.tm_mday = day;
    givenTime.tm_hour = hour;
    givenTime.tm_min  = min;
    givenTime.tm_sec  = sec;

    time_t seconds = mktime(&givenTime);

    return seconds + sec100 * 0.01;
}


/*----------------------------------------------------------------------
 * return the printStr 
 * the printStr will be cleaned after it is readed
 * through this function
 *--------------------------------------------------------------------*/
string DVL::getPrintString() {
    string newStr =  printStr;
    printStr = "";
    return newStr;
}

/* return the data string created by the decoder */
string DVL::getDataString() {
    return decoder.getPrintStr();
}
