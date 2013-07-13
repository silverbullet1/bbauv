#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <bbauv_msgs/openups.h>
#include <bbauv_msgs/openups_stats.h>

const int NUM_UPS = 4;
const float LOW_VOLTAGE = 21.6;

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

	bbauv_msgs::openups_stats openupsStatsMsg;
	int8_t *statsCharges = &openupsStatsMsg.charge1;
	float *statsCurrents = &openupsStatsMsg.current1;
	float *statsVoltages = &openupsStatsMsg.voltage1;

	ros::Publisher pubCharges = n.advertise<bbauv_msgs::openups>("openups_charge", 1);
	ros::Publisher pubStats = n.advertise<bbauv_msgs::openups_stats>("openups_stats", 1);
	ros::Rate loop_rate(2); // Hz

	//HACK: publish charge at a slower rate
	int counter = 0;

	while (ros::ok())
	{
		for (int ups_id=1; ups_id<=NUM_UPS; ++ups_id)
		{
			int index = ups_id - 1;

			ostringstream os;
			os << "upsc openups" << ups_id << "@localhost 2> /dev/null";
			FILE *fp = popen(os.str().c_str(), "r");

			statsCharges[index] = charges[index] = -3;
			statsCurrents[index] = 0;
			statsVoltages[index] = 0;

			if (fp)
			{
				const int MAX_BUFFER = 10000;
				char tmp[MAX_BUFFER] = { 0 };
				fread(tmp, 1, MAX_BUFFER-1, fp);
				string upsOutput(tmp);

				size_t found = upsOutput.find("Data stale");
				if (found != string::npos) {
					statsCharges[index] = charges[index] = -1;
				} else {
					string status = extractBatteryState<string>(upsOutput, "ups.status", "");

					//HACK: if "DISCHRG" not found in status, return default value
					float voltage = extractBatteryState<float>(upsOutput, "battery.voltage", -1);

					if (status.find("DISCHRG") == string::npos) {
						statsCharges[index] = charges[index] = -2;
					} else {
						charges[index] = extractBatteryState<int>(upsOutput, "battery.charge", -1);
						charges[index] = (voltage < LOW_VOLTAGE) ? 0 : charges[index];

						statsCharges[index] = charges[index];
						statsCurrents[index] = extractBatteryState<float>(upsOutput, "output.current", -1);
						statsVoltages[index] = voltage;
					}
				}
//				ROS_INFO("openups%d: %d %.3lfA %dsec %.2lfV\n", ups_id, (int)charges[ups_id-1]);
			}
			pclose(fp);
		}
		if (counter == 0)
			pubCharges.publish(openupsMsg);
		pubStats.publish(openupsStatsMsg);

		counter = (counter+1) % 5;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
