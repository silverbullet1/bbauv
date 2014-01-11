#include <ros/ros.h>
#include "bbauv_msgs/compass_data.h"
//#include <bbauv_msgs/controller_input>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

//#include "linefollowingstates.h"

using namespace std;

class LineFollower
{
public:
	LineFollower();

	//void bottomCamCallback(const bbauv_msgs::controller_input& msg);
	//void compassCallback(const bbauv_msgs::compass_data& msg);
	//void publishMovement(const bbauv_msgs::controller_input& movement);
	double normHeading(double heading);	
	void bottomCamCallback(const sensor_msgs::ImageConstPtr& msg);

private: 

	static const int endTime = 0;
	static const int DEPTH_POINT = 1.1;
	static const double secondsToRun = 2.25 * 60;

	ros::NodeHandle nh;
	//Subscribers to respective topic
	//Please write a .launch file to change the topic names if required
	ros::Subscriber imageSub;
	ros::Subscriber compassSub;
	ros::Publisher movementPub;
};

LineFollower::LineFollower()
{
// 	imageSub = nh.subscribe("/bumblebee/bottomcam", 1, &LineFollower::bottomCamCallback);
// 	//compassSub = nh.subscribe("/compass", 1, compassCallback);
// 	//movementPub = nh.advertise<bbauv_msgs::controller_input>("/movement", 1);
}

int kfd = 0;
struct termios cooked;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "linefollower");
	int loopRate = 20;
	ros::Rate loop_rate(loopRate);
	LineFollower linefollower;

	signal(SIGINT, quit);

	ros::spinOnce();
	loop_rate.sleep();

	return (0);
}

//Function to normalise heading in degrees
double LineFollower::normHeading(double heading)
{
	if (heading > 360.0) { return heading - 360.0; }
	else if (heading < 0.0) { return heading + 360.0; }
	else { return heading; }
}

//Having some problems here... 
// LineFollower::publishMovement(const bbauv_msgs::controller_input& movement){
// 	movementPub.publish(movement);
// }