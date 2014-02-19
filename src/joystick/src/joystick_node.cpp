#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <bbauv_msgs/ControllerAction.h>
#include <bbauv_msgs/ControllerGoal.h>
#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/depth.h>
#include <bbauv_msgs/set_controller.h>

#define JOY_DEV "/dev/input/js0"

/* Macros to map buttons/axes names to numbers */
// Axes
#define DPAD_X 6
#define DPAD_Y 7
#define LEFT_STICK_X 0
#define LEFT_STICK_Y 1
#define RIGHT_STICK_X 3
#define RIGHT_STICK_Y 4
#define LEFT_TRIGGER 2
#define RIGHT_TRIGGER 5
// Buttons
#define BUTTON_X 2
#define BUTTON_Y 3
#define BUTTON_A 0
#define BUTTON_B 1
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5
#define BACK_BUTTON 6
#define START_BUTTON 7
/* ------------------------------------------ */

ros::Subscriber compassSub;
ros::Subscriber depthSub;
actionlib::SimpleActionClient<bbauv_msgs::ControllerAction>* locoClient;

double curHeading = 0.0;
double curDepth = 0.0;
bool isHovering = true;

void initializeCom();
void compassCallback(const bbauv_msgs::compass_dataConstPtr& data);
void depthCallback(const bbauv_msgs::depthConstPtr& data);
// Function to send relative movement
void sendMovement(double f, double sm, double heading, double depth);

void enable();
void disable();
const int axisBound = 20000;
void handleEvent(int* axes, char* button);

double norm(double angle);

int main(int argc, char** argv) {
	ros::init(argc, argv, "joystick_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	initializeCom();
	enable();

	int joy_fd;
    int num_of_axis = 0, num_of_buttons = 0;
	int *axis = NULL;
	char *button = NULL;
	char name_of_joystick[80];
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
//		printf("Axis: \n");
//		for (int a = 0; a < num_of_axis; a++) {
//			printf("A%d: %d, ", a, axis[a]);
//		}
//
//		printf("Button: \n");
//		for (int b = 0; b < num_of_buttons; b++) {
//			printf("B%d: %d  ", b, button[b]);
//		}
//
//		printf("\n");

		handleEvent(axis, button);

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

double norm(double angle) {
	if (angle > 360.0) { return angle - 360.0; }
	else if (angle < 0.0) { return angle + 360.0; }
	else { return angle; }
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

void sendMovement(double f=0.0, double sm=0.0, double h=0.0, double d=0.0) {
	bbauv_msgs::ControllerGoal goal;
	goal.forward_setpoint = f;
	goal.sidemove_setpoint = sm;
	goal.heading_setpoint = norm(curHeading + h);
	goal.depth_setpoint = std::max(0.0, curDepth + d);

	locoClient->sendGoal(goal);
	printf("Moving f: %lf, sm: %lf, d: %lf, h: %lf\n", f, sm, d, h);
	locoClient->waitForResult(ros::Duration(0.5));
}

void enable() {
	ros::NodeHandle nh;
	ros::ServiceClient controlClient = nh.serviceClient<bbauv_msgs::set_controller>("set_controller_srv");
	bbauv_msgs::set_controller srv;
	srv.request.depth = true;
	srv.request.forward = true;
	srv.request.heading = true;
	srv.request.pitch = true;
	srv.request.roll= true;
	srv.request.sidemove = true;
	controlClient.call(srv);
}

void disable() {
	ros::NodeHandle nh;
	ros::ServiceClient controlClient = nh.serviceClient<bbauv_msgs::set_controller>("/set_controller_srv");
	bbauv_msgs::set_controller srv;
	srv.request.depth = false;
	srv.request.forward = false;
	srv.request.heading = false;
	srv.request.pitch = false;
	srv.request.roll= false;
	srv.request.sidemove = false;
	controlClient.call(srv);
}

void handleEvent(int* axes, char* button) {
	bool toHover = true;
	double f=0.0, sm=0.0, d=0.0, h=0.0;

	if (button[LEFT_BUTTON] == 1) {
		disable();
		isHovering = true;
		return;
	} else if (button[RIGHT_BUTTON] == 1) {
		enable();
	}

	if (axes[DPAD_Y] < -axisBound) {
		f = 0.5;
		toHover = false;
	} else if (axes[DPAD_Y] > axisBound) {
		f = -0.5;
		toHover = false;
	}

	if (axes[DPAD_X] > axisBound) {
		sm = 0.5;
		toHover = false;
	} else if (axes[DPAD_X] < -axisBound) {
		sm = -0.5;
		toHover = false;
	}

	if (button[BUTTON_Y] == 1) {
		d = 0.1;
		toHover = false;
	} else if (button[BUTTON_A] == 1) {
		d = -0.1;
		toHover = false;
	}

	if (button[BUTTON_B] == 1) {
		h = 10.0;
		toHover = false;
	} else if (button[BUTTON_X] == 1) {
		h = -10.0;
		toHover = false;
	}

	if (toHover && !isHovering) {
		isHovering = true;
		sendMovement(f, sm, h, d);
	} else if (!toHover) {
		isHovering = false;
		sendMovement(f, sm, h, d);
	}

//	printf("isHovering: %d\n", isHovering);
//	printf("Moving f: %lf, sm: %lf, d: %lf, h: %lf\n", f, sm, d, h);
}
