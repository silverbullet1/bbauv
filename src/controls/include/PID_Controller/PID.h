/*
 * PID.h
 *
 *  Created on: May 9, 2013
 *      Author: gohew
 */
#ifndef PID_H_
#define PID_H_

#include <ros/ros.h>

namespace bbauv {

class bbPID {
public:
	bbPID(std::string,double,double,double,int);

	void setKp(double P);
	void setTi(double I);
	void setTd(double D);
	double getProportional();
	double getDerivative();
	double getTotal();
	double getIntegral();
	void setActuatorSatModel(int min,int max);
	double actuatorConstrain(double val);
	double computePID(double setpoint, double input);
	double wrapAngle360(double error, double heading);
	void clearIntegrator();
	virtual ~bbPID();
private:
	//User parameters
	double Kp;
	double Ti;
	double Td;
	double Imax;
	double N;
	int actMax;
	int actMin;
	//Controller variables
	double _proportional;
	double _derivative;
	double _total;
	ros::Time oldTime;
	double inputOld;
	double _integral;
	std::string _name;
};

} /* namespace bbauv */
#endif /* PID_H_ */
