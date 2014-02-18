#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <signal.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <bbauv_msgs/ControllerAction.h>
#include <bbauv_msgs/ControllerGoal.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/depth.h>

#define JOY_DEV "/dev/input/js0"

ros::Subscriber compassSub;
ros::Subscriber depthSub;
actionlib::SimpleActionClient<bbauv_msgs::ControllerAction>* locoClient;

double curHeading = 0.0;
double curDepth = 0.0;

void initializeCom();
void compassCallback(const bbauv_msgs::compass_dataConstPtr& data);
void depthCallback(const bbauv_msgs::depthConstPtr& data);
// function to send relative movement
void sendMovement(double f, double sm, double heading, double depth);
void handleButton(char* button, char* prevButton, int numButtons);
void handleAxis(int* axes, int* prevAxes, int numAxes);

int main(int argc, char** argv) {
	ros::init(argc, argv, "joystick_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	initializeCom();

	int joy_fd, *axis = NULL, num_of_axis = 0, num_of_buttons = 0, x;
	char *button = NULL, name_of_joystick[80];
	struct js_event js;

	if ((joy_fd = open( JOY_DEV, O_RDONLY)) == -1) {
		printf("Couldn't open joystick\n");
		return -1;
	}

	ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
	ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
	ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

	axis = (int *) calloc(num_of_axis, sizeof(int));
	button = (char *) calloc(num_of_buttons, sizeof(char));

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n",
			name_of_joystick, num_of_axis, num_of_buttons);

	fcntl(joy_fd, F_SETFL, O_NONBLOCK); /* use non-blocking mode */

	ros::Rate rate(20);
	while (ros::ok()) { /* read the joystick state */
		read(joy_fd, &js, sizeof(struct js_event));

		/* see what to do with the event */
		switch (js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
		}

		/* print the results */
		printf("Axis: \n");
		for (int a = 0; a < num_of_axis; a++) {
			printf("A%d: %d, ", a, axis[a]);
		}

		printf("Button: \n");
		for (int b = 0; b < num_of_buttons; b++) {
			printf("B%d: %d  ", b, button[b]);
		}

		printf("\n");
		fflush(stdout);

		ros::spinOnce();
		rate.sleep();
	}

	// Cleaning up stuffs
	delete locoClient;
	free (axis);
	free (button);
	close(joy_fd);
	return 0;
}

void initializeCom() {
	ros::NodeHandle nh;
	compassSub = nh.subscribe("/euler", 1, compassCallback);
	depthSub = nh.subscribe("/depth", 1, depthCallback);

	locoClient = new actionlib::SimpleActionClient<bbauv_msgs::ControllerAction>("/LocomotionServer", true);
	printf("Waiting for Locomotion Server...\n");
	bool waitResult = locoClient->waitForServer(ros::Duration(5));
	if (waitResult) {
		printf("Locomotion Server ready!\n");
	} else {
		printf("Fail to connect to Locomotion Server!");
	}
}

void compassCallback(const bbauv_msgs::compass_dataConstPtr& data) {
	curHeading = data->yaw;
}

void depthCallback(const bbauv_msgs::depthConstPtr& data) {
	curDepth = data->depth;
}

void sendMovement(double f=0.0, double sm=0.0, double heading=0.0, double depth=0.0) {
	bbauv_msgs::ControllerGoal goal;
	goal.forward_setpoint = f;
	goal.sidemove_setpoint = sm;
	goal.heading_setpoint = curHeading + heading;
	goal.depth_setpoint = curDepth + depth;

	locoClient->sendGoal(goal);
	locoClient->waitForResult(ros::Duration(0.5));
}
