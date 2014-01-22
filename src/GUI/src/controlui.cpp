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
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
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
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "controlui_add.h"
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

static QVector<double> graph_x(101), graph_setpt(101), graph_output(101);

class ControlUI {
private:
	ros::NodeHandle private_nh;

	void initialiseDefault();
	void initializeGraph();

	void subscribeToData();

	//Helper functions to update Dynamic Reconfigure params
	void updateParameter(string paramName, bool val);
	void updateParameter(string paramName, double val);
	void updateParameter(string paramName, int val);

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

	map<string, string> params; //Map for parameters

	//Variables for DoFs
	float depth;
	float yaw, pitch, roll;
	float x, y;

	float depth_kp;
	float yaw_kp, pitch_kp, roll_kp;
	float x_kp, y_kp;

	float depth_ki;
	float yaw_ki, pitch_ki, roll_ki;
	float x_ki, y_ki;

	float depth_kd;
	float yaw_kd, pitch_kd, roll_kd;
	float x_kd, y_kd;

	double error;

	//For graph
	string graphType;
	double graphOut, graphSetPt;
	double x_org;

	void initialiseParameters();

	void thruster_val_callback(const bbauv_msgs::thruster::ConstPtr& msg);

	void controllerPointsCallBack(const bbauv_msgs::controller::ConstPtr& data);

	void errorCallBack(const bbauv_msgs::ControllerActionFeedbackConstPtr& feedback);
};
ControlUI* controlUI;

ControlUI::ControlUI() : nh(), private_nh("~"), live(true), enable(false) {
	startTime = ros::Time::now();
	private_nh.param("live", live, bool(false));

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

	graphOut = 0.0;
	graphSetPt = 0.0;
	graphType = "Depth";

	initialiseParameters();
	initializeGraph();
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

//To subscribe to data topics: currently under default!!
void ControlUI::subscribeToData() {
	//Note: Setpoint get from Dynamic Reconfigure server / Parameter Server

	//Thrusters Subscriber [thrusters.msg]
	thruster_sub = nh.subscribe("/thruster_speed", 1, &ControlUI::thruster_val_callback, this);

	errorSub = nh.subscribe("/LocomotionServer/feedback", 1, &ControlUI::errorCallBack, this);

	//Graph controller points
	graph_update = nh.subscribe("/controller_points", 1, &ControlUI::controllerPointsCallBack, this);
}

/*
	Helper functions to update Dynamic Reconfigure params
*/
void ControlUI::updateParameter(string paramName, bool val) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::BoolParameter bool_param;
	dynamic_reconfigure::Config conf;

	bool_param.name = paramName;
	bool_param.value = val;
	conf.bools.push_back(bool_param);

	srv_req.config = conf;

	ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
}

void ControlUI::updateParameter(string paramName, double val) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;

	double_param.name = paramName;
	double_param.value = val;
	conf.doubles.push_back(double_param);

	srv_req.config = conf;

	ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
}

void ControlUI::updateParameter(string paramName, int val) {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::IntParameter integer_param;
	dynamic_reconfigure::Config conf;

	integer_param.name = paramName;
	integer_param.value = val;
	conf.ints.push_back(integer_param);

	srv_req.config = conf;

	ros::service::call("/Controller/set_parameters", srv_req, srv_resp);
}

void ControlUI::initialiseDefault() {
	params.clear();

	params["depth_Kp"] = "";
	params["depth_Ti"] = "";
	params["depth_Td"] = "";
	params["depth_min"] = "";
	params["depth_max"] = "";

	params["pitch_Kp"] = "";
	params["pitch_Ti"] = "";
	params["pitch_Td"] = "";
	params["pitch_min"] = "";
	params["pitch_max"] = "";

	params["roll_Kp"] = "";
	params["roll_Ti"] = "";
	params["roll_Td"] = "";
	params["roll_min"] = "";
	params["roll_max"] = "";

	params["heading_Kp"] = "";
	params["heading_Ti"] = "";
	params["heading_Td"] = "";
	params["heading_min"] = "";
	params["heading_max"] = "";

	params["forward_Kp"] = "";
	params["forward_Ti"] = "";
	params["forward_Td"] = "";
	params["forward_min"] = "";
	params["forward_max"] = "";

	params["sidemove_Kp"] = "";
	params["sidemove_Ti"] = "";
	params["sidemove_Td"] = "";
	params["sidemove_min"] = "";
	params["sidemove_max"] = "";
}

void ControlUI::initialiseParameters() {
	//Telemetry part
	ui.KP_val->setText(params.find("depth_Kp")->second.c_str());
	ui.KI_val->setText(params.find("depth_Ti")->second.c_str());
	ui.KD_val->setText(params.find("depth_Td")->second.c_str());

	//Controls part
	ui.con_KP_val->setText(params.find("depth_Kp")->second.c_str());
	ui.con_KI_val->setText(params.find("depth_Ti")->second.c_str());
	ui.con_KD_val->setText(params.find("depth_Td")->second.c_str());
	ui.actmin_val->setText(params.find("depth_min")->second.c_str());
	ui.actmax_val->setText(params.find("depth_max")->second.c_str());

}

void ControlUI::controllerPointsCallBack(const bbauv_msgs::controller::ConstPtr& data) {
	if (graphType == "Depth") {
		graphSetPt = data->depth_setpoint;
		graphOut = data->depth_input;
		//ROS_INFO("%lf", graphOut);
	} else if (graphType == "Heading") {
		graphSetPt = data->heading_setpoint;
		graphOut = data->heading_input;
		//ROS_INFO("%lf", graphOut);
	} else if (graphType == "Forward") {
		graphSetPt = data->forward_setpoint;
		graphOut = data->forward_input;
		//ROS_INFO("%lf", graphOut);
	} else if (graphType == "Side") {
		graphSetPt = data->sidemove_setpoint;
		graphOut = data->sidemove_input;
		//ROS_INFO("%lf", graphOut);
	} else if (graphType == "Roll") {
		graphSetPt = data->roll_setpoint;
		graphOut = data->roll_input;
		//ROS_INFO("%lf", graphOut);
	} else if (graphType == "Pitch") {
		graphSetPt = data->pitch_setpoint;
		graphOut = data->pitch_input;
		//ROS_INFO("%lf", graphOut);
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

//////////////////////////
// Non Class functions //
/////////////////////////

void updateStatus() {
	controlUI->ui.error_val->setText(boost::lexical_cast<string>(controlUI->error).c_str());
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
	//controlUI->ui.graph_canvas->graph(1)->rescaleAxes();
	controlUI->ui.graph_canvas->replot();

	controlUI->ui.setpt_val->setText(boost::lexical_cast<std::string>(controlUI->graphSetPt).c_str());
	controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->graphOut).c_str());
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
	file << "setpt_val " << controlUI->ui.setpt_val->text().toUtf8().constData() << "\n";
	file << "sensor_val " << controlUI->ui.sensor_val->text().toUtf8().constData() << "\n";
	file << "error_val " << controlUI->ui.error_val->text().toUtf8().constData()<< "\n";
	file << "KP_val " << controlUI->ui.KP_val->text().toUtf8().constData() << "\n";
	file << "KI_val " << controlUI->ui.KI_val->text().toUtf8().constData() << "\n";
	file << "KD_val " << controlUI->ui.KD_val->text().toUtf8().constData()<< "\n";

	file << "goal_val " << controlUI->ui.goal_val->text().toUtf8().constData() << "\n";

	if ( controlUI->ui.fwd_check->isChecked() ) {
		file << "fwd_check " << "true" << "\n";
	} else {
		file << "fwd_check " << "false " << "\n";
	}
	file << "fwd_val "  << controlUI->ui.fwd_val->text().toUtf8().constData() << "\n";

	if ( controlUI->ui.depth_check->isChecked() ){
		file << "depth_check " << "true" << "\n";
	} else {
		file << "depth_check " << "false " << "\n";
	}
	file << "depth_val "  << controlUI->ui.depth_val->text().toUtf8().constData() << "\n";

	if ( controlUI->ui.yaw_check->isChecked() ){
		file << "yaw_check " << "true" << "\n";
	} else {
		file << "yaw_check " << "false " << "\n";
	}
	file << "yaw_val " << controlUI->ui.yaw_val->text().toUtf8().constData() << "\n";

	if ( controlUI->ui.sm_check->isChecked() ){
		file << "sm_check " << "true" << "\n";
	} else {
		file << "sm_check " << "false " << "\n";
	}
	file << "sm_val "  << controlUI->ui.sm_val->text().toUtf8().constData() << "\n";

	file << "con_KP_val " << controlUI->ui.con_KP_val->text().toUtf8().constData() << "\n";
	file << "con_KI_val " << controlUI->ui.con_KI_val->text().toUtf8().constData() << "\n";
	file << "con_KD_val " << controlUI->ui.con_KD_val->text().toUtf8().constData()<< "\n";
	file << "actmin_val " << controlUI->ui.actmin_val->text().toUtf8().constData() << "\n";
	file << "actmax_val " << controlUI->ui.actmax_val->text().toUtf8().constData()<< "\n";

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
		ros::NodeHandle nh;
		float goal_val = atof(controlUI->params.find("goal_val")->second.c_str());;
		std_msgs::Float32 msg;
		ros::Publisher goal_pub = nh.advertise<std_msgs::Float32>("goal_pub", 1);
		msg.data = goal_val;
		goal_pub.publish(msg);
		ros::spinOnce();
	}
}

// Send the advanced parameters 
void sendButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Send!", "Bang! Boom! Bam!");
	}
	else{
		ros::NodeHandle nh;
		actionlib::SimpleActionClient <bbauv_msgs::ControllerAction> ac ("LocomotionServer", true);
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");

		//Send goal and publish to topics to controller.msgs
		bbauv_msgs::ControllerGoal goal; 
		double temp;

		bool forward, sidemove, heading, depth, pitch, roll;
		heading = controlUI->ui.yaw_check->isChecked();
		forward = controlUI->ui.fwd_check->isChecked();
		depth = controlUI->ui.depth_check->isChecked();
		sidemove = controlUI->ui.sm_check->isChecked();
		pitch = false;
		roll = false;

	    ros::ServiceClient controlClient = nh.serviceClient<bbauv_msgs::set_controller>("set_controller_srv");

	    bbauv_msgs::set_controller srv;
	    srv.request.depth = depth;
	    srv.request.forward = forward;
	    srv.request.heading = heading;
	    srv.request.pitch = pitch;
	    srv.request.roll= roll;
	    srv.request.sidemove = sidemove;

	    controlClient.call(srv);

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

		ac.sendGoal(goal);
		ros::spinOnce();

		bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));
		if (finished_before_timeout){
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
			QMessageBox::information(controlUI->ui.centralwidget, "Goal status", state.toString().c_str());
		}
		else {
			ROS_INFO("Timeout! Goal not achieved");
			QMessageBox::critical(controlUI->ui.centralwidget, "Goal status", "Timeout! Goal not achieved");
		}
	}
}

// Tune the control parameters by publishing them
void tuneButton(){
	if (!controlUI->live){
		QMessageBox::information(controlUI->ui.centralwidget, "Tune!", "Twinkle Twinkle Little Star~");
	}
	else {
		float temp;
		std_msgs::Float32 msg;

		ros::Publisher con_KP_pub = controlUI->nh.advertise<std_msgs::Float32>("con_KP_pub", 1);
		temp = atof(controlUI->params.find("con_KP_val")->second.c_str());
		msg.data = temp;
		con_KP_pub.publish(msg);

		ros::Publisher con_KD_pub = controlUI->nh.advertise<std_msgs::Float32>("con_KD_pub", 1);
		temp = atof(controlUI->params.find("con_KD_val")->second.c_str());
		msg.data = temp;
		con_KD_pub.publish(msg);

		ros::Publisher con_KI_pub = controlUI->nh.advertise<std_msgs::Float32>("con_KI_pub", 1);
		temp = atof(controlUI->params.find("con_KI_val")->second.c_str());
		msg.data = temp;
		con_KI_pub.publish(msg);

		ros::Publisher actmin_pub = controlUI->nh.advertise<std_msgs::Float32>("actmin_pub", 1);
		temp = atof(controlUI->params.find("actmin_val")->second.c_str());
		msg.data = temp;
		actmin_pub.publish(msg);

		ros::Publisher actmax_pub = controlUI->nh.advertise<std_msgs::Float32>("actmax_pub", 1);
		temp = atof(controlUI->params.find("actmax_val")->second.c_str());
		msg.data = temp;
		actmax_pub.publish(msg);
	}
}

void graphTypeChanged(const QString& type) {
	controlUI->ui.graph_canvas->graph(0)->clearData();
	controlUI->ui.graph_canvas->graph(1)->clearData();
	controlUI->x_org = 0;
	controlUI->startTime = ros::Time::now();
	controlUI->graphType = type.toStdString();
}
