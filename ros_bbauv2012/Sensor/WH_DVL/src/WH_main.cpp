/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node, get DVL data and use callbacks to
 * publish DVL data.
 * Package : WH_DVL                  Node   : WH_DVL
 * Classes : Serial > DVL > RDI_DVL  Object : rdi_dvl
 * Config  : WH_DVL.cfg <WH_DVL, WH_DVL>  
 *----------------------------------------------------------------------------*/

#include <iostream>
#include <fstream>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>

// Local includes.
#include "WH_ros.h"
#include "WH_core.h"
#include "timing.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <WH_DVL/WH_DVLConfig.h>

using namespace std;

int main(int argc, char **argv)
{
    bool connection_lost = false;
    string printString;

    ofstream myfile;
    myfile.open("rawData.txt");

    // Set up ROS.
    ros::init(argc, argv, "WH_DVL");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    // Local variables
    int baud;
    int init_time;
    string port_name;
    string odom_topic_name;
    string alt_topic_name;

    // Initialize node parameters.
    private_node_handle_.param("baud", baud, int(57600));
    private_node_handle_.param("init_time", init_time, int(5));
    private_node_handle_.param("port", port_name, string("/dev/ttyDVL"));
    private_node_handle_.param("odom_topic_name", odom_topic_name, string("WH_DVL_data"));
    private_node_handle_.param("alt_topic_name", alt_topic_name, string("altitude"));

    // Create a new WH_DVL object.
    RDI_DVL *rdi_dvl = new RDI_DVL(port_name, baud, init_time);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<WH_DVL::WH_DVLConfig> gain_srv;
    dynamic_reconfigure::Server<WH_DVL::WH_DVLConfig>::CallbackType f;
    f = boost::bind(&RDI_DVL::configCallback, rdi_dvl,  _1, _2);
    gain_srv.setCallback(f);

    // Set up publishers.
    ros::Publisher pubOdomData = n.advertise<nav_msgs::Odometry>(odom_topic_name.c_str(), 100);
    ros::Publisher pubAltitudeData = n.advertise<std_msgs::Float32>(alt_topic_name.c_str(), 100);

    // Tell ROS to run this node at the rate that the DVL is sending messages to us.
    ros::Rate r(rdi_dvl->ros_rate);
    ROS_INFO("ros_rate: %d", rdi_dvl->ros_rate);

    // Connect to the Workhorse DVL
    if (rdi_dvl->fd < 0) {
        ROS_ERROR("Could not connect to DVL on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", rdi_dvl->portname.c_str(), rdi_dvl->baud);
    }

    // Main loop.
    while (n.ok()) {
        // Get DVL data.
        if (rdi_dvl->fd > 0) {
            if (connection_lost){
                connection_lost = false; 
                rdi_dvl->zeroDistance();
            }

            if (rdi_dvl->cmdMode) {
                rdi_dvl->getRawData();
                printString = rdi_dvl->getPrintString();
                if (printString != "") {
                    cout << "WH::DVL is recording data" << endl;
                    myfile << printString;
                    cout << printString;
                }
            }
            else {
                rdi_dvl->getData();
                /*
                if (rdi_dvl->getPrintString() != "")
                    cout << rdi_dvl->getPrintString();
                */

                //Publish the message.
                rdi_dvl->publishOdomData(&pubOdomData);
                rdi_dvl->publishAltitudeData(&pubAltitudeData);
            }
        }

        else {
            connection_lost = true;
            ROS_ERROR("WH::DVL Connection is lost. fd = %d", rdi_dvl->fd);
        }

        ros::spinOnce();
        r.sleep();
    }

    myfile.close();
    return 0;
} // end main()
