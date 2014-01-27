/*
 * flarestates.h
 *
 *  Created on: 24 Jan, 2014
 *      Author: ndt
 */

#ifndef FLARESTATES_H_
#define FLARESTATES_H_

#include <ros/ros.h>
#include <bbauv_msgs/controller.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
extern const int DEPTH_POINT;
extern ros::Publisher movementPub;

inline void publishMovement(const bbauv_msgs::controller& movement){
	movementPub.publish(movement);
}

//Structure for bounding box
struct RectData{
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Abstract base class for states
class State : public boost::enable_shared_from_this<State>{
public:
	State() {};
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

//Look for flare state
class LookForFlareState : public State{
public:
	LookForFlareState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//Lost flare state
class LostFlareState : public State{
public:
	LostFlareState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//Surface
class SurfaceState : public State {
public:
	SurfaceState(double heading);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

#endif /* FLARESTATES_H_ */
