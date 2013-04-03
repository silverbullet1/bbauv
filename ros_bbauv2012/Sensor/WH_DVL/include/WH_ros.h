//WH_ros.h

#ifndef WH_ROS_H
#define WH_ROS_H

#include <iostream>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// Local includes.
#include "WH_core.h"
#include "decoder.h"
#include "serial.h"
#include "timing.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <WH_DVL/WH_DVLConfig.h>

using namespace std;

/******************************
 *
 * Classes
 *
 *****************************/

class RDI_DVL : public DVL
{
private:
    bool verifying;
    //bool calibrating;

public:
    bool cmdMode;
    int ros_rate;
    //! Constructor.
    RDI_DVL(string _portname, int _baud, int _init_time);

    //! Destructor.
    ~RDI_DVL();

    // publish data in Odometry message
    void publishOdomData(ros::Publisher *pubData);

    //! Callback function for dynamic reconfigure server.
    void configCallback(WH_DVL::WH_DVLConfig &config, uint32_t level);
}; //end class WH_DVL

#endif // WH_ROS_H
