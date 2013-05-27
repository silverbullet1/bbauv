#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <PID_Controller/ControllerAction.h>
#include <boost/thread.hpp>

void spinThread()
{
   ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    ROS_INFO("Client is doing other crap");
    ros::spinOnce();
    r.sleep();
  }
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const PID_Controller::ControllerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  //ROS_INFO("Answer: %i", result->sequence.back());
}

// Called once when the goal completes
void done2Cb(const actionlib::SimpleClientGoalState& state,
            const PID_Controller::ControllerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  //ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const PID_Controller::ControllerFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %i", feedback->thruster);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_averaging");

  // create the action client
  actionlib::SimpleActionClient<PID_Controller::ControllerAction> ac("SwayAction",true);
  actionlib::SimpleActionClient<PID_Controller::ControllerAction> ac2("ForwardAction",true);

  //boost::thread spin_thread(&spinThread);
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

 ROS_INFO("Action server started, sending goal.");
 ros::Rate r(1);
  while(ros::ok())
  {

	  // send a goal to the action
	 PID_Controller::ControllerGoal goal;
	 //goal.setpoint = 10;
	 ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	   //wait for the action to return
	  bool finished_before_timeout = ac.waitForResult(ros::Duration(30));
	   if (finished_before_timeout)
	  {
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	  }
	  else
	    ROS_INFO("Action did not finish before the time out.");
	  //goal.setpoint = 200;
	  ac2.sendGoal(goal, &done2Cb);
	  finished_before_timeout = ac2.waitForResult(ros::Duration(30));

	  ROS_INFO("Client is doing other crap");


	  ros::spinOnce();
	  r.sleep();
  }
  // shutdown the node and join the thread back before exiting
  //ros::shutdown();
  //spin_thread.join();

  //exit
  return 0;
}
