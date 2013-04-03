/*----------------------------WH_ros.cpp----------------------------------------
 * output message: nav_msgs/Odometry
 * dynamic reconfigure: yes
 ------------------------------------------------------------------------------*/



#include "WH_ros.h"

/*----------------DVL() - Constructor---------------------------------*/
RDI_DVL::RDI_DVL(string _portname, int _baud, int _init_time) : DVL::DVL(_portname, _baud, _init_time) {
    ros_rate    = 2;
    cmdMode     = true;
    verifying   = false;
    //calibrating = false;
}


/*----------------~DVL() - Destructor---------------------------------*/
RDI_DVL::~RDI_DVL() {
}

/*----------------publish odometry message----------------------------*/
void RDI_DVL::publishOdomData(ros::Publisher *pubData) {
    nav_msgs::Odometry odomData;

    //time stamp
    //ros time starts in 1970
    ros::Time givenTime(totalSec);
    odomData.header.stamp = givenTime;

    // coordinate frames
    odomData.header.frame_id = header_frame_id;
    odomData.child_frame_id  = child_frame_id;

    // FILL IN POSE DATA
    // position
    odomData.pose.pose.position.x = x;
    odomData.pose.pose.position.y = y;
    odomData.pose.pose.position.z = z;

    // orientation
    odomData.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angX*M_PI/180.0, angY*M_PI/180.0, angZ*M_PI/180.0);
    ROS_DEBUG("DVL quaternions = %.1f, %.1f, %.1f, %.1f", 
            odomData.pose.pose.orientation.x, 
            odomData.pose.pose.orientation.y, 
            odomData.pose.pose.orientation.z, 
            odomData.pose.pose.orientation.w);

    // covariance matrix
    for ( int i=0; i < 36; i++)
        odomData.pose.covariance[i] = posCov[i];

    // FILL IN TWIST DATA
    // covariance matrix
    for ( int i=0; i < 36; i++)
        odomData.twist.covariance[i] = velCov[i];

    odomData.twist.twist.angular.z = angZvel;
    odomData.twist.twist.angular.x = angXvel;
    odomData.twist.twist.angular.y = angYvel;

    odomData.twist.twist.linear.x  = xvel;
    odomData.twist.twist.linear.y  = yvel;
    odomData.twist.twist.linear.z  = zvel;

    pubData->publish(odomData);
} // end publishOdomData()

/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void RDI_DVL::configCallback(WH_DVL::WH_DVLConfig &config, uint32_t level) {
    // Set class variables to new values.
    init_time = config.init_time;
    portname  = config.port.c_str();
    ros_rate  = config.ros_rate;

    // send start pinging (CS) command
    if (config.pinging == true && cmdMode == true) {
        ROS_INFO("Start pinging");
        startPinging();
        cmdMode = false;
    }

    cmdMode = !config.pinging;

    // send the custom command to the DVL
    if (config.command != "") {
        cmdMode = true;
        sendCommand(config.command, true);
        ROS_INFO("command %s is sent to the DVL", config.command.c_str());
        config.command = "";
    }

    // send break, clean other work that being performed
    if (config.send_break == true) {
        ROS_INFO("Break is sent");
        sendBreak();
        cmdMode           = true;
        config.send_break = false;
    }

    // zero the travelled distance
    if (config.zero_distance == true) {
        ROS_INFO("Zero the travelled distance");
        zeroDistance();
        config.zero_distance = false;
    }

    // verify the compass data
    if (config.verify_compass == true && verifying == false) {
        ROS_INFO("Verify compass sent");
        cmdMode = true;
        verifyCompass();
    }
    else if (config.verify_compass == false && verifying == true) {
        ROS_INFO("Stop verifying compass sent");
        stopVerifying();
    }
    verifying = config.verify_compass;

    /*
    // CANNOT CONTINUE, NO POINT
    // start the calibration
    if (config.calibrate == true && calibrating == false) {
        ROS_INFO("Calibrate compass sent");
        cmdMode = true;
        calibrateCompass();
    }
    else if (config.calibrate == false && calibrating == true) {
        ROS_INFO("Stop calibrating compass");
        cmdMode = true;
        stopCalibrating();
    }
    calibrating = config.calibrate;
    */

    // print out the full data set
    if (config.print_data == true) {
        ROS_INFO("Printing data");
        cout << getDataString() << endl;
        config.print_data = false;
    }

    // synchronize the DVL internal clock with the system clock
    if (config.clock_sync == true) {
        ROS_INFO("Synchronize clock sent");
        cmdMode = true;
        setTSClock();
        config.clock_sync = false;
    }

    /*
    // TOO DANGEROUS OPTION
    // reset to factory settings then send all standard settings again
    if (config.reset_settings == true) {
        ROS_INFO("Reset to basic settings");
        cmdMode = true;
        resetSettings();
        config.reset_settings = false;
    }
    */

    // Check to see if we should attempt to reconnect to the compass.
    if (config.reconnect) {
        // Use the new compass settings to reconnect.
        cmdMode = true;
        setup();
        ROS_INFO("Using new settings to reconnect to DVL. Got fd = %d", fd);

        // Reset the reconnect variable.
        config.reconnect = false;
    }

    config.pinging = !cmdMode;
} // end configCallback()
