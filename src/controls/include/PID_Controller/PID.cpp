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

/****************** Getter Functions ***********************/

double bbPID::getTotal()
{
	 return _total;
}

double bbPID::getProportional()
{
	 return _proportional;
}

double bbPID::getIntegral()
{
	 return _integral;
}

double bbPID::getDerivative()
{
	 return _derivative;
}

void bbPID::setActuatorSatModel(int min=-1000,int max=1000)
{
	actMax = max;
	actMin = min;
}

//Constructor
bbPID::bbPID(std::string name, double P,double I,double D, int Ncut) {
	_derivative = 0;
	_integral = 0;
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
	double output;
	double Tt = sqrt(Ti*Td);
	if(oldTime.nsec > nowTime.nsec) dt = (nowTime.nsec + 1000000000 - oldTime.nsec)/1000000;
	else	dt = (nowTime.nsec - oldTime.nsec)/1000000;

	//if(_name == "h") _proportional = Kp*(fmod((setpoint - input),180));
	_proportional = Kp*(setpoint - input);

	//This implements the derivative with set point weighting and bandwidth limitation
	//for filtering of noisy signals. Equivalent to low pass filters.
	_derivative = (Td/(Td + N*dt))*(_derivative - Kp*N*(input - inputOld));
	_total = _proportional + _derivative + _integral;

	output = actuatorConstrain(_total);
	ROS_DEBUG("n: %s P: %2.f, I: %2.f, D: %2.f, dt: %2.2f, err: %6.2f",_name.c_str(),_proportional,_integral, _derivative,dt,setpoint - input);

	//std::cout<<" P: "<<proportional<<" D: "<<derivative<<" I: "<<integral<<std::endl;
	//std::cout<<"output: "<<output<<std::endl;
	//Integrator with wind up protection
	if(Ti) _integral = _integral + Kp*dt*(setpoint - input)/Ti + (output - _total)*dt/Tt;
	else _integral = 0;
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
	_integral = 0;
}
bbPID::~bbPID() {
	// TODO Auto-generated destructor stub
}

} /* namespace bbauv */
