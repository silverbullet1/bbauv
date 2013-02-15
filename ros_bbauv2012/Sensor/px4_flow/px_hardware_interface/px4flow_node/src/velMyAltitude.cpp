#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <bbauv_msgs/controller_input.h>
#include <px_comm/OpticalFlow.h>
#include <px_comm/velMyAltitude.h>

using namespace std;

/* Global Variable declaration */

double Max_Depth;
double Depth;
double Magnifying_Factor;
double Flow_X;
double Flow_Y;

/* Function prototypes */

void updateDepthInput(const bbauv_msgs::controller_input &msg);
void updateFlow(const px_comm::OpticalFlow &msg);

/* ROS Initialization */

bbauv_msgs::controller_input ctrl;
px_comm::velMyAltitude vel;
ros::Subscriber ctrl_input;
ros::Subscriber flow_input;
ros::Publisher velMyAltitude;

/************ Main Loop *****************/

int main(int argc,char** argv)
{

    ros::init(argc,argv,"velMyAltitude");
    ros::NodeHandle nh;

    //set up subscriber
    ctrl_input = nh.subscribe("/controller_input", 20, &updateDepthInput);
    flow_input = nh.subscribe("/px4flow/opt_flow",20,&updateFlow);

    //set up publisher
    velMyAltitude = nh.advertise<px_comm::velMyAltitude>("/velMyAltitude",20);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
   
    //get Params
    nh.getParam("/px4flow/Max_Depth",Max_Depth);
    nh.getParam("/px4flow/Magnifying_Factor",Magnifying_Factor);

    //Flow and Altitude Scale Calculations
    
    vel.header.stamp = ros::Time::now();
    vel.velocity_x = Flow_X * (( (Max_Depth-Depth)*Magnifying_Factor )/0.016);
    vel.velocity_y = Flow_Y * (( (Max_Depth-Depth)*Magnifying_Factor )/0.016);

    velMyAltitude.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();

    }
    return 0;
}

/* ROS Callback functions */

void updateDepthInput (const bbauv_msgs::controller_input &msg)
{
    Depth=msg.depth_input;
}

void updateFlow (const px_comm::OpticalFlow &msg)
{
    Flow_X = msg.flow_x;
    Flow_Y = msg.flow_y;
}

