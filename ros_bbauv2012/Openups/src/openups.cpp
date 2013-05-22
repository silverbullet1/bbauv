#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <bbauv_msgs/openups.h>

const int NUM_UPS = 4;

using namespace std;

// Template function to extract property values from the battery state output
// by the OpenUPS driver.
//
// Known properties are:
// - battery.charge  (int)
// - battery.current (float)
// - battery.runtime (int)
// - battery.voltage (float)
//
// Returns defaultVal if the property cannot be found.
template<class T>
T extractBatteryState(const string& upsOutput, const string& property, const T& defaultVal)
{
	string searchStr = property + ": ";
	size_t found = upsOutput.find(searchStr);
	if (found != string::npos) {
		T value;
		istringstream sstr(upsOutput.substr(found+searchStr.length()));
		sstr >> value;
		return value;
	}
	return defaultVal;
}
template<>
string extractBatteryState<string>(const string& upsOutput, const string& property, const string& defaultVal)
{
	string searchStr = property + ": ";
	size_t found = upsOutput.find(searchStr);
	if (found != string::npos) {
		string value;
		istringstream sstr(upsOutput.substr(found+searchStr.length()));
		getline(sstr, value, '\n');
		return value;
	}
	return defaultVal;
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "openupsPublisher");
	ros::NodeHandle n;
	bbauv_msgs::openups openupsMsg;
	int8_t *charges = &openupsMsg.battery1;

	ros::Publisher pub = n.advertise<bbauv_msgs::openups>("openups", 1);
	ros::Rate loop_rate(0.2);

	while (ros::ok())
	{
		for (int ups_id=1; ups_id<=NUM_UPS; ++ups_id)
		{
			ostringstream os;
			os << "upsc openups" << ups_id << "@localhost 2> /dev/null";
			FILE *fp = popen(os.str().c_str(), "r");

			if (fp)
			{
				const int MAX_BUFFER = 10000;
				char tmp[MAX_BUFFER] = { 0 };
				fread(tmp, 1, MAX_BUFFER-1, fp);
				string upsOutput(tmp);

				string status = extractBatteryState<string>(upsOutput, "ups.status", "");

				//HACK: if "DISCHRG" not found in status, return default value
				if (status.find("DISCHRG") == string::npos) {
					charges[ups_id-1] = -1;
				} else {
					charges[ups_id-1] = extractBatteryState<int>(upsOutput, "battery.charge", -1);
				}

//				ROS_INFO("openups%d: %d %.3lfA %dsec %.2lfV\n", ups_id, (int)charges[ups_id-1]);
			}
			pclose(fp);
		}
		pub.publish(openupsMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
