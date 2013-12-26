//WH_ros.h

#ifndef WH_ROS_H
#define WH_ROS_H

#include <iostream>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

// Local includes.
#include <WH_core.h>

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <sensors/WH_DVLConfig.h>

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

    // publish datas
    void publishOdomData(ros::Publisher *pubOdomData);
    void publishAltitudeData(ros::Publisher *pubAltitudeData);

    //! Callback function for dynamic reconfigure server.
    void configCallback(sensors::WH_DVLConfig &config, uint32_t level);
}; //end class WH_DVL

#endif // WH_ROS_H
