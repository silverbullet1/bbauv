/*
 * PID.cpp
 *
 *  Created on: May 9, 2013
 *      Author: gohew
 */

#include "PID.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

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

void bbPID::setActuatorSatModel(int min=0,int max=1000)
{
	actMax = max;
	actMin = 0;
}

//Constructor
bbPID::bbPID(double P,double I,double D, int Ncut) {
	Kp = P;
	Ti = I;
	Td = D;
	N = Ncut;
}

double bbPID::computePID(double setpoint, double input)
{
	ros::Time nowTime = ros::Time::now();
	double dt = nowTime.nsec - oldTime.nsec;
	double output;
	if(oldTime.nsec > nowTime.nsec) dt = (nowTime.nsec + 1000000000 - oldTime.nsec)/1000000;
	else	dt = (nowTime.nsec - oldTime.nsec)/1000000;

	double proportional = Kp*(setpoint - input);

	//This implements the deri_Controller/PID.h>vative with set point weighting and bandwidth limitation
	//for filtering of noisy signals. Equivalent to low pass filters.
	derivative = (Td/(Td + N*dt))*(derivative - Kp*N*(inputOld - input)/(Td + N*dt));
	integral = integral + Kp*dt*(setpoint - input)/Ti;

	output = actuatorConstrain(proportional + derivative + integral);

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

	return constrain_val;
}
bbPID::~bbPID() {
	// TODO Auto-generated destructor stub
}

} /* namespace bbauv */
