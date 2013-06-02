/*
 * PID.cpp
 *
 *  Created on: May 9, 2013
 *      Author: gohew
 */

#include "PID.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

namespace bbauv {

//Setters

void bbPID::setKp(double P)
{
	Kp = P;
}

void bbPID::setTi(double I)
{
	Ti = I;
}

void bbPID::setTd(double D)
{
	Td = D;
}

void bbPID::setActuatorSatModel(int min=-1000,int max=1000)
{
	actMax = max;
	actMin = min;
}

//Constructor
bbPID::bbPID(std::string name, double P,double I,double D, int Ncut) {
	derivative = 0;
	integral = 0;
	Kp = P;
	Ti = I;
	Td = D;
	N = Ncut;
	_name = name;
}

double bbPID::computePID(double setpoint, double input)
{
	ros::Time nowTime = ros::Time::now();
	double dt = nowTime.nsec - oldTime.nsec;
	double output,total;
	double Tt = sqrt(Ti*Td);
	if(oldTime.nsec > nowTime.nsec) dt = (nowTime.nsec + 1000000000 - oldTime.nsec)/1000000;
	else	dt = (nowTime.nsec - oldTime.nsec)/1000000;

	double proportional = Kp*(setpoint - input);

	//This implements the derivative with set point weighting and bandwidth limitation
	//for filtering of noisy signals. Equivalent to low pass filters.
	derivative = (Td/(Td + N*dt))*(derivative - Kp*N*(input - inputOld));
	total = proportional + derivative + integral;

	output = actuatorConstrain(total);
	ROS_DEBUG("n: %s P: %2.f, I: %2.f, D: %2.f",_name.c_str(),proportional,integral, derivative);
	//std::cout<<" P: "<<proportional<<" D: "<<derivative<<" I: "<<integral<<std::endl;
	//std::cout<<"output: "<<output<<std::endl;
	//Integrator with wind up protection
	if(Ti) integral = integral + Kp*dt*(setpoint - input)/Ti + (output - total)*dt/Tt;
	else integral = 0;
	//Update old time and input for derivative computation
	oldTime = nowTime;
	inputOld = input;
	return output;
}

double bbPID::wrapAngle360(double error, double heading)
{
	if (error > 180)
	{
		heading +=360;
	}
	else if (error < -180)
	{
		heading -= 360;
	}

	return heading;
}

double  bbPID::actuatorConstrain(double val)
{
	double constrain_val = 0;
	if(val > actMax)	constrain_val = actMax;
	else if(val < actMin) constrain_val = actMin;
	else constrain_val = val;

	return constrain_val;
}

void bbPID::clearIntegrator()
{
	integral = 0;
}
bbPID::~bbPID() {
	// TODO Auto-generated destructor stub
}

} /* namespace bbauv */
