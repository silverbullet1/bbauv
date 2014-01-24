/* 
	controlui.cpp
	A GUI for tuning the control systems
	Date created: 8 Jan 2014
	Author: Jason & Lynnette
*/

#include "controlui.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <bbauv_msgs/controller.h>
#include <bbauv_msgs/ControlData.h>
#include <bbauv_msgs/ControllerAction.h>
#include <bbauv_msgs/ControllerGoal.h>
#include <bbauv_msgs/thruster.h>
#include <bbauv_msgs/set_controller.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/ReconfigureResponse.h>

#include <qstring.h>
#include <qtimer.h>
#include <qvector.h>

#include <ros/ros.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "controlui_add.h"

//Convenient stuffs for dynamic reconfiguring
#define numParams 30
string dynamicParams[31] =
{
"depth_Kp", "depth_Ti", "depth_Td", "depth_min", "depth_max",
"pitch_Kp", "pitch_Ti",	"pitch_Td", "pitch_min", "pitch_max",
"roll_Kp", "roll_Ti", "roll_Td", "roll_min", "roll_max",
"heading_Kp", "heading_Ti", "heading_Td", "heading_min", "heading_max",
"forward_Kp", "forward_Ti", "forward_Td", "forward_min", "forward_max",
"sidemove_Kp", "sidemove_Ti", "sidemove_Td", "sidemove_min", "sidemove_max"
};

//index for each DOF in the dynamicParams array
int depthIndex = 0, pitchIndex = 5, rollIndex = 10,
	headingIndex = 15, forwardIndex = 20, sidemoveIndex = 25;

//Types for each DOF params
string paramsTypes[] = {"double_t", "double_t", "double_t", "int_t", "int_t"};

std::string roundString(double val, int precision) {
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(precision);
	ss << val;
	std::string s = ss.str();

	return s;
}

class ControlUI {
private:
	ros::NodeHandle private_nh;

	void initialiseDefault();
	void initializeGraph();

	void subscribeToData();

	//Subscribers
	ros::Subscriber graph_update;
	ros::Subscriber thruster_sub;
	ros::Subscriber errorSub;
public:
	ControlUI();

	ros::NodeHandle nh;

	Ui::ControlSysUI ui;
	QMainWindow *window;
	ros::Time startTime;

	bool live;
	bool enable;

	map<string, string> params; //Map for dynamic parameters

	double error;

	//For graph
	string graphType;
	int startIndex, endIndex;
	double graphOut, graphSetPt;
	double x_org;

	void initialiseParameters();
	void autoSave();
	void loadControlParams();

	//Data callbacks
	void thruster_val_callback(const bbauv_msgs::thruster::ConstPtr& msg);
	void controllerPointsCallBack(const bbauv_msgs::controller::ConstPtr& data);
	void errorCallBack(const bbauv_msgs::ControllerActionFeedbackConstPtr& feedback);

	//Helper functions to update Dynamic Reconfigure params
	void updateParameter(string paramName, bool val, dynamic_reconfigure::Config&);
	void updateParameter(string paramName, double val, dynamic_reconfigure::Config&);
	void updateParameter(string paramName, int val, dynamic_reconfigure::Config&);

	QLineEdit* configureWidgets[5];
};
ControlUI* controlUI;

ControlUI::ControlUI() : nh(), private_nh("~"), live(true), enable(false) {
	startTime = ros::Time::now();
	private_nh.param("live", live, bool(true));

	initialiseDefault();

	if (!live){
		ROS_INFO("%s", "Not going live");
	} else {
		ROS_INFO("%s", "Going live!");
		subscribeToData();
	}

	window = new QMainWindow;
	ui.setupUi(window);
	window->setFixedSize(window->geometry().width(), window->geometry().height());
	configureWidgets[0] = ui.conKpVal;
	configureWidgets[1] = ui.conTiVal;
	configureWidgets[2] = ui.conTdVal;
	configureWidgets[3] = ui.conMinVal;
	configureWidgets[4] = ui.conMaxVal;

	graphOut = 0.0;
	graphSetPt = 0.0;
	graphType = "Depth";

	startIndex = 0;
	endIndex = 5;

	initialiseParameters();
	initializeGraph();
}

void ControlUI::errorCallBack(const bbauv_msgs::ControllerActionFeedbackConstPtr& feedback){
	if (graphType == "Heading") {
		error = feedback->feedback.heading_error;
	} else if (graphType == "Forward") {
		error = feedback->feedback.forward_error;
	} else if (graphType == "Side") {
		error = feedback->feedback.sidemove_error;
	} else if (graphType == "Depth") {
		error = feedback->feedback.depth_error;
	} else {
		error = 0;
	}
}

void ControlUI::subscribeToData() {
	//Thrusters Subscriber [thrusters.msg]
	thruster_sub = nh.subscribe("/thruster_speed", 1, &ControlUI::thruster_val_callback, this);

	errorSub = nh.subscribe("/LocomotionServer/feedback", 1, &ControlUI::errorCallBack, this);

	//Graph controller points
	graph_update = nh.subscribe("/controller_points", 1, &ControlUI::controllerPointsCallBack, this);
}

//Helper functions to update Dynamic Reconfigure params
void ControlUI::updateParameter(string paramName, bool val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::BoolParameter bool_param;

	bool_param.name = paramName;
	bool_param.value = val;
	conf.bools.push_back(bool_param);
}

void ControlUI::updateParameter(string paramName, double val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;

	double_param.name = paramName;
	double_param.value = val;
	conf.doubles.push_back(double_param);

	srv_req.config = conf;

	ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
}

void ControlUI::updateParameter(string paramName, int val, dynamic_reconfigure::Config& conf) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::IntParameter integer_param;

	integer_param.name = paramName;
	integer_param.value = val;
	conf.ints.push_back(integer_param);

	srv_req.config = conf;

	ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
}

void ControlUI::initialiseDefault() {
	params.clear();

	for (int i = 0; i < numParams; i++) {
		params[dynamicParams[i]] = "0.0";
	}
}

void ControlUI::initialiseParameters() {
	FILE* input;
	input = fopen("controlParams.txt", "r");
	if (input != NULL) {
		for (int i = 0; i < numParams; i++) {
			char val[256];
			fscanf(input, (dynamicParams[i] + ": %s\n").c_str(), val);
			params[dynamicParams[i]] = string(val);
		}

		fclose(input);
	}

	//Controls part
	ui.conKpVal->setText(params.find("depth_Kp")->second.c_str());
	ui.conTiVal->setText(params.find("depth_Ti")->second.c_str());
	ui.conTdVal->setText(params.find("depth_Td")->second.c_str());
	ui.conMinVal->setText(params.find("depth_min")->second.c_str());
	ui.conMaxVal->setText(params.find("depth_max")->second.c_str());

}

void ControlUI::autoSave() {
	FILE* output;
	output = fopen("controlParams.txt", "w");
	for (int i = 0; i < numParams; i++) {
		string curParam = dynamicParams[i];
		fprintf(output, (curParam + ": %s\n").c_str(), params[curParam].c_str());
	}
	fclose(output);
}

void ControlUI::controllerPointsCallBack(const bbauv_msgs::controller::ConstPtr& data) {
	if (graphType == "Depth") {
		graphSetPt = data->depth_setpoint;
		graphOut = data->depth_input;
	} else if (graphType == "Heading") {
		graphSetPt = data->heading_setpoint;
		graphOut = data->heading_input;
	} else if (graphType == "Forward") {
		graphSetPt = data->forward_setpoint;
		graphOut = data->forward_input;
	} else if (graphType == "Side") {
		graphSetPt = data->sidemove_setpoint;
		graphOut = data->sidemove_input;
	} else if (graphType == "Roll") {
		graphSetPt = data->roll_setpoint;
		graphOut = data->roll_input;
	} else if (graphType == "Pitch") {
		graphSetPt = data->pitch_setpoint;
		graphOut = data->pitch_input;
	}
}

//To plot the graph of sensors and setpt
void ControlUI::initializeGraph() {
	//Make legend visible
	//ui.graph_canvas->legend->setVisible(true);
	//ui.graph_canvas->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignRight|Qt::AlignBottom);

	//Add the graphs
	ui.graph_canvas->addGraph();
	ui.graph_canvas->graph(0)->setName("Setpt");
	ui.graph_canvas->graph(0)->setPen(QPen(Qt::red));

	ui.graph_canvas->addGraph();
	ui.graph_canvas->graph(1)->setName("Output");
	ui.graph_canvas->graph(1)->setPen(QPen(Qt::blue));

	// give the axes some labels:
	ui.graph_canvas->xAxis->setLabel("Time (s)");

	//For user interaction
	ui.graph_canvas->setInteraction(QCP::iRangeDrag, true);
	ui.graph_canvas->setInteraction(QCP::iRangeZoom, true);
	QObject::connect(ui.graph_canvas, &QCustomPlot::mouseMove, mouseMoved);

	x_org = 0;

	//Plot the graph
	ui.graph_canvas->replot();
}

void ControlUI::thruster_val_callback (const bbauv_msgs::thruster::ConstPtr& msg) {
	ui.thruster_val_1->setText(QString::number(msg->speed1));
	ui.thruster_val_2->setText(QString::number(msg->speed2));
	ui.thruster_val_3->setText(QString::number(msg->speed3));
	ui.thruster_val_4->setText(QString::number(msg->speed4));
	ui.thruster_val_5->setText(QString::number(msg->speed5));
	ui.thruster_val_6->setText(QString::number(msg->speed6));
	ui.thruster_val_7->setText(QString::number(msg->speed7));
	ui.thruster_val_8->setText(QString::number(msg->speed8));
}

void ControlUI::loadControlParams() {
	for (int i = startIndex; i < endIndex; i++) {
		int offsetIndex = i % 5;
		configureWidgets[offsetIndex]->setText(QString::fromStdString(params[dynamicParams[i]]));
	}
}

//////////////////////////
// Non Class functions //
/////////////////////////

void updateStatus() {
	controlUI->ui.error_val->setText(QString::fromStdString(roundString(controlUI->error, 4)));
}

void updateGraph() {
	double x_val = (ros::Time::now() - controlUI->startTime).toSec();

	double x_org = controlUI->x_org;
	if (x_val - x_org > 20) {
		controlUI->ui.graph_canvas->graph(0)->removeDataBefore(x_org + 1);
		controlUI->ui.graph_canvas->graph(1)->removeDataBefore(x_org + 1);
		controlUI->x_org++;
	}

	controlUI->ui.graph_canvas->graph(0)->addData(x_val, controlUI->graphSetPt);//Set Point
	controlUI->ui.graph_canvas->graph(1)->addData(x_val, controlUI->graphOut);//Output
	controlUI->ui.graph_canvas->rescaleAxes();
	controlUI->ui.graph_canvas->replot();

	controlUI->ui.setpt_val->setText(QString::fromStdString(roundString(controlUI->graphSetPt, 4)));
	controlUI->ui.sensor_val->setText(QString::fromStdString(roundString(controlUI->graphOut, 4)));
}

string getdate() {
	int max_date = 12;
	time_t now;
	char date[max_date];
	date[0] = '\0';
	now = time(NULL);
	if (now != -1) { strftime(date, max_date, "%d%m%y", gmtime(&now)); }
	return date;
}

void saveFile() {
	//Open a .txt file
	string filename = "controls";
	filename.append("_");
	filename.append(getdate());
	filename.append(".txt");
	ofstream file;
	file.open(filename.c_str());

	file << "con_KP_val " << controlUI->ui.conKpVal->text().toUtf8().constData() << "\n";
	file << "con_KI_val " << controlUI->ui.conTiVal->text().toUtf8().constData() << "\n";
	file << "con_KD_val " << controlUI->ui.conTdVal->text().toUtf8().constData()<< "\n";
	file << "actmin_val " << controlUI->ui.conMinVal->text().toUtf8().constData() << "\n";
	file << "actmax_val " << controlUI->ui.conMaxVal->text().toUtf8().constData()<< "\n";

	file.close();
	QMessageBox::information(controlUI->ui.centralwidget, "File saved", "File successfully saved! :)");
}

void openTheFile() {
	string line;
	vector<string> tokens;
	QString selfilter = QString("*.bag");
	QString filename = QFileDialog::getOpenFileName(controlUI->window, QString("Open controls file"), QDir::currentPath(),
	QString(".txt files (*.txt);; All files (*.*)"), &selfilter);

	string filename_string = filename.toUtf8().constData();
	ifstream file;
	file.open(filename_string.c_str());
	if (file.is_open()){
		string msg = "Loading file " + filename_string + "...";
		ROS_INFO("%s", msg.c_str());

		while (getline(file, line)) {
			boost::algorithm::split(tokens, line, boost::algorithm::is_any_of(" "));
			controlUI->params[tokens[0]] = tokens[1];
		}

		controlUI->initialiseParameters();
		file.close();
		}
	else {
		QMessageBox::critical(controlUI->ui.centralwidget, "Error", "Could not open file");
		return;
	} 	
}

//To enable all the check boxes
void enableButton(){
	controlUI->enable = !controlUI->enable;
	if (controlUI->enable){
		controlUI->ui.fwd_check->setChecked(true);
		controlUI->ui.depth_check->setChecked(true);
		controlUI->ui.yaw_check->setChecked(true);
		controlUI->ui.sm_check->setChecked(true);
	}
	else {
		controlUI->ui.fwd_check->setChecked(false);
		controlUI->ui.depth_check->setChecked(false);
		controlUI->ui.yaw_check->setChecked(false);
		controlUI->ui.sm_check->setChecked(false);
	}
}

//Mouse clicked on graph so display data point
void mouseMoved(QMouseEvent *event) {
	ostringstream oss;
	oss << std::fixed << std::setprecision(3);

	double x = controlUI->ui.graph_canvas->xAxis->pixelToCoord(event->x());
 	double y = controlUI->ui.graph_canvas->yAxis->pixelToCoord(event->y());

 	oss << "Graph values: x: " << x << "\ty: " << y ;
 	controlUI->ui.graphvalues->setText(QString::fromStdString(oss.str()));
}

//To send goal value
void fire() {
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Fire!", "Bang! Boom! Bam!");
	} else {
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::Config conf;

		double value = controlUI->ui.goal_val->text().toDouble();
		if (controlUI->graphType == "Depth") {
			controlUI->updateParameter("depth_setpoint", value, conf);
		} else if (controlUI->graphType == "Pitch") {
			controlUI->updateParameter("pitch_setpoint", value, conf);
		} else if (controlUI->graphType == "Heading") {
			controlUI->updateParameter("heading_setpoint", value, conf);
		} else if (controlUI->graphType == "Roll") {
			controlUI->updateParameter("roll_setpoint", value, conf);
		} else if (controlUI->graphType == "Side") {
			controlUI->updateParameter("sidemove_setpoint", value, conf);
		} else if (controlUI->graphType == "Forward") {
			controlUI->updateParameter("forward_setpoint", value, conf);
		}

		srv_req.config = conf;
		ros::service::call("/Controller/set_parameters", srv_req, srv_resp);

		ros::spinOnce();
	}
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const bbauv_msgs::ControllerResultConstPtr& result) {
	const char* status = state.toString().c_str();
	ROS_INFO("Finished in state [%s]", status);
	controlUI->ui.statusLabel->setText(status);
}

// Called once when the goal becomes active
void activeCb() {
  ROS_INFO("Goal just went active");
  controlUI->ui.statusLabel->setText("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const bbauv_msgs::ControllerFeedbackConstPtr& feedback) {
	//ROS_INFO("Got Feedback of length %i", feedback->thruster);
}

// Send the advanced parameters
void sendButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Send!", "Bang! Boom! Bam!");
	}
	else {
		ros::NodeHandle nh;

		bool forward, sidemove, heading, depth, pitch, roll;
		heading = controlUI->ui.yaw_check->isChecked();
		forward = controlUI->ui.fwd_check->isChecked();
		depth = controlUI->ui.depth_check->isChecked();
		sidemove = controlUI->ui.sm_check->isChecked();
		pitch = controlUI->ui.pitch_check->isChecked();
		roll = controlUI->ui.roll_check->isChecked();

	    ros::ServiceClient controlClient = nh.serviceClient<bbauv_msgs::set_controller>("set_controller_srv");

	    bbauv_msgs::set_controller srv;
	    srv.request.depth = depth;
	    srv.request.forward = forward;
	    srv.request.heading = heading;
	    srv.request.pitch = pitch;
	    srv.request.roll= roll;
	    srv.request.sidemove = sidemove;

	    controlClient.call(srv);

		actionlib::SimpleActionClient <bbauv_msgs::ControllerAction> ac ("LocomotionServer", true);
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");

		//Send goal and publish to topics to controller.msgs
		bbauv_msgs::ControllerGoal goal;

		if (heading) {
			goal.heading_setpoint = controlUI->ui.yaw_val->text().toDouble();
		} else {
			goal.heading_setpoint = 0.0;
		}

		if (forward) {
			goal.forward_setpoint = controlUI->ui.fwd_val->text().toDouble();
		} else {
			goal.forward_setpoint = 0.0;
		}

		if (depth) {
			goal.depth_setpoint = controlUI->ui.depth_val->text().toDouble();
		} else {
			goal.depth_setpoint = 0.0;
		}

		if (sidemove){
			goal.sidemove_setpoint = controlUI->ui.sm_val->text().toDouble();
		} else {
			goal.sidemove_setpoint = 0.0;
		}

		ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
		ros::spinOnce();

//		bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));
//		if (finished_before_timeout){
//			actionlib::SimpleClientGoalState state = ac.getState();
//			ROS_INFO("Action finished: %s", state.toString().c_str());
//		}
//		else {
//			ROS_INFO("Timeout! Goal not achieved");
//		}
	}
}

// Tune the control parameters by publishing them
void tuneButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Tune!", "Twinkle Twinkle Little Star~");
	} else {
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::Config conf;

		for (int i = controlUI->startIndex; i < controlUI->endIndex; i++) {
			int offsetIndex = i % 5;
			if (paramsTypes[offsetIndex] == "int_t") {
				int temp = controlUI->configureWidgets[offsetIndex]->text().toInt();
				controlUI->updateParameter(dynamicParams[i], temp, conf);
				controlUI->params[dynamicParams[i]] = boost::lexical_cast<string, int>(temp);
			} else if (paramsTypes[offsetIndex] == "double_t") {
				double temp = controlUI->configureWidgets[offsetIndex]->text().toDouble();
				controlUI->updateParameter(dynamicParams[i], temp, conf);
				controlUI->params[dynamicParams[i]] = boost::lexical_cast<string, double>(temp);
			}
		}

		srv_req.config = conf;
		ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
		controlUI->autoSave();
		controlUI->ui.statusLabel->setText("Status: Finished Tuning");
	}
}

void graphTypeChanged(const QString& type) {
	controlUI->ui.graph_canvas->graph(0)->clearData();
	controlUI->ui.graph_canvas->graph(1)->clearData();
	controlUI->x_org = 0;
	controlUI->startTime = ros::Time::now();
	controlUI->graphType = type.toStdString();

	if (controlUI->graphType == "Depth") {
		controlUI->startIndex = depthIndex;
	} else if (controlUI->graphType == "Heading") {
		controlUI->startIndex = headingIndex;
	} else if (controlUI->graphType == "Forward") {
		controlUI->startIndex = forwardIndex;
	} else if (controlUI->graphType == "Side") {
		controlUI->startIndex = sidemoveIndex;
	} else if (controlUI->graphType == "Roll") {
		controlUI->startIndex = rollIndex;
	} else if (controlUI->graphType == "Pitch") {
		controlUI->startIndex = pitchIndex;
	}
	controlUI->endIndex = controlUI->startIndex + 5; //Exclusive

	controlUI->loadControlParams();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);

	ControlUI lControlUI;
	controlUI = &lControlUI;

	QObject::connect(controlUI->ui.actionSave, &QAction::triggered, saveFile);
	QObject::connect(controlUI->ui.fireButton, &QAbstractButton::released, fire);
	QObject::connect(controlUI->ui.actionOpen, &QAction::triggered, openTheFile);
	QObject::connect(controlUI->ui.enabledButton, &QAbstractButton::released, enableButton);
	QObject::connect(controlUI->ui.tuneButton, &QAbstractButton::released, tuneButton);
	QObject::connect(controlUI->ui.sendButton, &QAbstractButton::released, sendButton);
	QObject::connect(controlUI->ui.graphType,
					 static_cast<void (QComboBox::*)(const QString&)>(&QComboBox::currentIndexChanged),
					 graphTypeChanged);

	QTimer *timer = new QTimer();
	QObject::connect(timer, &QTimer::timeout, updateGraph);
	timer->start(50);

	QTimer *statusTimer = new QTimer();
	QObject::connect(statusTimer, &QTimer::timeout, updateStatus);
	statusTimer->start(50);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	controlUI->window->show();

	int status = app.exec();
	delete timer;
	delete statusTimer;
	return status;
}
