#include <stdio.h>

#include <stdlib.h>

#include <vector>

#include <iostream>

#include "ros/ros.h"
#include "bbauv_msgs/circles.h"
#include <vector>

using namespace std;

vector< float > x;
vector< float > y;
vector< float > radius;


void arrayCallback(const bbauv_msgs::circles::ConstPtr& array){
	ROS_INFO("CALL BACK");
	x.clear();
	y.clear();
	radius.clear();
	int size=array->size;
	std::vector<float>::const_iterator Xit=array->x.begin();
	std::vector<float>::const_iterator Yit=array->y.begin();
	std::vector<float>::const_iterator Rit=array->radius.begin();
	for (int i=0;i<size;i++){
		float XX=*Xit;
		x.push_back(XX);
		y.push_back(*Yit);
		radius.push_back(*Rit);
		Xit++;Yit++;Rit++;
	}
	for (int i=0;i<size;i++){
		cout<<"id "<<i<<" X Y Radius "<< x[i]<<" "<<y[i]<<" "<<radius[i]<<endl;
	}
	
} 
int main(int argc, char **argv){
	
	ros::init(argc, argv, "CircleSubscriber");
	ros::NodeHandle n;	

	ROS_INFO("Prepare to subscribe ");
 
	ros::Subscriber sub3 = n.subscribe("array", 100, arrayCallback);
	ros::spin();

	return 0;
}


