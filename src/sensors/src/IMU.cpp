/*
 * IMU.cpp
 *  Porting SpartonCompassIMU.py to a C++ code
 *  Created on: 16 Jan, 2014
 *      Author: huixian
 */

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <ros/time.h>

//For messaging
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "bbauv_msgs/imu_data.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

//Import tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

static const double GYRO_SAMPLE_RATE = 107.95;

struct D_compass{
	std::string port;
	int baud, printmodulus;
	double offset;
};

class IMU{
public:
	IMU();
	~IMU();

	D_compass compass;

	double wrapTo2PI(double theta);
	double wrapToPI(double theta);
	void publishData();

private:
	ros::NodeHandle nh;

	//Subscribers
	ros::Publisher IMU_pub_q;
	ros::Publisher IMU_pub_e;
	ros::Publisher IMU_pub_temp;

};

IMU::IMU(){
	IMU_pub_q = nh.advertise<sensor_msgs::Imu>("/AHRS8_data_q", 1);
	IMU_pub_e = nh.advertise<bbauv_msgs::imu_data>("/AHRS8_data_e", 1);
	IMU_pub_temp = nh.advertise<std_msgs::Float32>("/AHRS8_Temp", 1);
}

IMU::~IMU(){
	std::string myStr1 = "";
	ROS_INFO("Sparton shutdown time!");
}

IMU *imu;

int main(int argc, char** argv){
	ros::init(argc, argv, "SpartonDigitalCompassIMU");
	ROS_INFO("Initialising Digital Compas IMU");

	IMU local_IMU;
	imu = &local_IMU;

	//Read information about compass in
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("~port", imu->compass.port, std::string("/dev/ttyUSB0"));
	private_node_handle.param("~baud", imu->compass.baud, int(115200));
	private_node_handle.param("~printmodulus", imu->compass.printmodulus, int(1));
	private_node_handle.param("~offset", imu->compass.offset, double(0.0));

	//Publish data
	imu->publishData();

	std::string myStr1="\r\n\r\nprinttrigger 0 set drop\r\n";
    std::string myStr2="printmask gyrop_trigger accelp_trigger or quat_trigger or yawt_trigger or time_trigger or temp_trigger or set drop\r\n";
    //std::string myStr_printmodulus = ("printmodulus %i set drop\r\n", imu->compass.printmodulus  );
    std::string myStr3 = "printtrigger printmask set drop\r\n";

	while(ros::ok()){
		ros::AsyncSpinner spinner(4);
		spinner.start();
		ros::waitForShutdown();
	}
	imu->~IMU();
	return(0);
}

//Normalise an anglie in radians to [0, 2*pi]
double IMU::wrapTo2PI(double theta){
	return fmod(theta, (2.0*M_PI));
}

//Normalise an angle in radians to [-pi, pi]
double IMU::wrapToPI(double theta){
	return (wrapTo2PI(theta+M_PI)-M_PI);
}

//Publish relevant data
void IMU::publishData(){
 //How to make this in cpp
  //Imu_data = Imu(header=rospy.Header(frame_id="AHRS8"))
//	  Imu_data.orientation_covariance = [1e-3, 0, 0,
//	                                       0, 1e-3, 0,
//	                                       0, 0, 1e-3]
//
//	    Imu_data.angular_velocity_covariance = [1e-3, 0, 0,
//	                                            0, 1e-3, 0,
//	                                            0, 0, 1e-3]
//
//	    Imu_data.linear_acceleration_covariance = [1e-3, 0, 0,
//	                                               0, 1e-3, 0,
//	                                               0, 0, 1e-3]

 ros::spinOnce();
}

