#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <bbauv_msgs/openups.h>


using namespace std;

int battery_charge(string s);
float battery_current(string s);
int battery_runtime(string s);
float battery_voltage(string s);
//string port(string s);

int main(int argc, char** argv)
{

	ros::init(argc, argv, "openupsPublisher");
	ros::NodeHandle n;
	bbauv_msgs::openups openupsMsg;

	ros::Publisher pub = n.advertise<bbauv_msgs::openups>("openups",1000);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{	
		system("upsc openups@localhost > tmp");
		ifstream infile("tmp");

		if (infile)
		{
			char s[10000];
			infile.get(s, 10000, 0);


			/*cout << "Battery charge is " << battery_charge(s) << std:: endl;
			  cout << "Battery current is " << battery_current(s) << std::endl;
			  cout << "Battery runtime is " << battery_runtime(s) << std::endl;
			  cout << "Battery voltage is " << battery_voltage(s) << std::endl;*/

			openupsMsg.batteryCharge=battery_charge(s);
			openupsMsg.batteryCurrent=battery_current(s);
			openupsMsg.batteryRuntime=battery_runtime(s);
			openupsMsg.batteryVoltage=battery_voltage(s);

			ROS_INFO("%d %.3lf %d %.2lf\n", openupsMsg.batteryCharge, openupsMsg.batteryCurrent, openupsMsg.batteryRuntime, openupsMsg.batteryVoltage);
		}
		pub.publish(openupsMsg);

		ros::spinOnce();
		loop_rate.sleep();

	}


	return 0;
}

int battery_charge(string s)
{
	string str ("battery.charge: ");
	size_t found;
	found=s.find(str);
	if(found!=string::npos)
	{
		int charge;
		istringstream sstr(s.substr(found+str.length()));
		sstr >> charge;
		return charge;
	}
}

float battery_current(string s)
{
	string str ("battery.current: ");
	size_t found;
	found=s.find(str);
	if(found!=string::npos)
	{
		float current;
		istringstream sstr(s.substr(found+str.length()));
		sstr >> current;
		return current;
	}
}  

int battery_runtime(string s)
{
	string str ("battery.runtime: ");
	size_t found;
	found=s.find(str);
	if(found!=string::npos)
	{
		int runtime;
		istringstream sstr(s.substr(found+str.length()));
		sstr >> runtime;
		return runtime;
	}
}  

float battery_voltage(string s)
{
	string str ("battery.voltage: ");
	size_t found;
	found=s.find(str);
	if(found!=string::npos)
	{
		float voltage;
		istringstream sstr(s.substr(found+str.length()));
		sstr >> voltage;
		return voltage;
	}
}  


