#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/sensors_data.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

string bagfile_path = "bag_files/2012-12-17-12-51-44.bag";
char textfile_path[] = "text_files/testing.txt";

int main(int argc, char** argv) {
	ros::init(argc,argv,"test");
	ros::NodeHandle n;
	rosbag::Bag bag;
	bag.open(bagfile_path, rosbag::bagmode::Read);
	vector<string> topics;
	topics.push_back(string("os5000_data"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	ofstream myfile;
	myfile.open(textfile_path);
	

	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		
		bbauv_msgs::compass_data::ConstPtr s=m.instantiate<bbauv_msgs::compass_data>();
		if (s!=NULL) {
			//myfile << (m.getTime()).toSec() << ", ";
			myfile << (ros::Time::now()-m.getTime()).toSec() << ", ";
			myfile << s->yaw << ", ";
			myfile << s->pitch << ", ";
			myfile << s->roll << ", ";
			myfile << s->temperature << ", ";
			myfile << s->Ax << ", ";
			myfile << s->Ay << ", ";
			myfile << s->Az << endl;
		}
	}
	bag.close();
	return 0;
}
