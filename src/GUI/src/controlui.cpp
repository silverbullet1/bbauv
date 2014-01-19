/* 
	controlui.cpp
	A GUI for tuning the control systems
	Date created: 8 Jan 2014
	Author: Jason & Lynnette
*/

#include "controlui_add.h"
#include "qcustomplot.h"

#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/ControlData.h>
#include <bbauv_msgs/controller.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

static QVector<double> graph_x(101), graph_setpt(101), graph_output(101);

class ControlUI {
private:
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	void initialiseDefault();
	void initializeGraph();

	void subscribeToData();

	//Helper functions to update Dynamic Reconfigure params
	void updateParameter(string paramName, bool val);
	void updateParameter(string paramName, double val);
	void updateParameter(string paramName, int val);

	void saveFile();
	void openFile();
public:
	ControlUI();

	enum GraphTypes { DEPTH, HEADING, FORWARD };

	Ui::ControlSysUI ui;
	QMainWindow *window;
	ros::Time startTime;

	bool live;
	bool enable;

	map <string, string> params; //Map for parameters

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

	//For graph
	double graphDepthOut;
	double graphDepthSetPt;
	double x_org;

	void initialiseParameters();

	//DoF Subscribers
	void depth_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);
	void yaw_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);
	void pitch_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);
	void roll_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);
	void x_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);
	void y_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg);

	//Functions for the subscribers
	void setpt_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void sensor_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void error_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void KD_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void output_val_callback(const std_msgs::Float32::ConstPtr& msg);

	void thruster_val_callback(const bbauv_msgs::thruster::ConstPtr& msg);

	void controllerPointsCallBack(const bbauv_msgs::controller data);

	void dof_val_callback(const std_msgs::String::ConstPtr& msg);
	void goal_val_callback(const std_msgs::Float32::ConstPtr& msg);

	void fwdcheck_callback(const std_msgs::Bool::ConstPtr& msg);
	void fwd_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void depthcheck_callback(const std_msgs::Bool::ConstPtr& msg);
	void depth_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void yawcheck_callback(const std_msgs::Bool::ConstPtr& msg);
	void yaw_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void smcheck_callback(const std_msgs::Bool::ConstPtr& msg);
	void sm_val_callback(const std_msgs::Float32::ConstPtr& msg);

	void actmin_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void actmax_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void con_KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void con_KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
	void con_KD_val_callback(const std_msgs::Float32::ConstPtr& msg);
};
ControlUI* controlUI;

//Utilities Functions
string getdate();

//Qt UI callbacks
void updateGraph();

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

	initialiseParameters();
	initializeGraph();
	subscribeToData();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);

	ControlUI lControlUI;
	controlUI = &lControlUI;

	QObject::connect(controlUI->ui.actionSave, &QAction::triggered, saveFile);
	QObject::connect(controlUI->ui.fireButton, &QAbstractButton::released, fire);
	QObject::connect(controlUI->ui.actionOpen, &QAction::triggered, openFile);
	QObject::connect(controlUI->ui.enabledButton, &QAbstractButton::released, enableButton);
	QObject::connect(controlUI->ui.tuneButton, &QAbstractButton::released, tuneButton);
	QObject::connect(controlUI->ui.sendButton, &QAbstractButton::released, sendButton);
	QObject::connect(controlUI->ui.dof_comboBox,
					 static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
					 dofSelected);

	QTimer *timer = new QTimer();
	QObject::connect(timer, &QTimer::timeout, updateGraph);
	timer->start(50);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	controlUI->window->show();

	return app.exec();
}

//To subscribe to data topics: currently under default!!
void ControlUI::subscribeToData() {
	//Note: Setpoint get from Dynamic Reconfigure server / Parameter Server

	//For telemetry
	ros::Subscriber setpt_val_sub = nh.subscribe("/Controller/DOF/a", 1, &ControlUI::setpt_val_callback, this);
	ros::Subscriber sensor_sub = nh.subscribe("/Controller/DOF/b", 1, &ControlUI::sensor_val_callback, this);
	ros::Subscriber error_sub = nh.subscribe("/Controller/DOF/c", 1, &ControlUI::error_val_callback, this);
	ros::Subscriber KP_val_sub = nh.subscribe("/Controller/DOF/d", 1, &ControlUI::KP_val_callback, this);
	ros::Subscriber KI_sub = nh.subscribe("/Controller/DOF/e", 1, &ControlUI::KI_val_callback, this);
	ros::Subscriber KD_sub = nh.subscribe("/Controller/DOF/f", 1, &ControlUI::KD_val_callback, this);
	ros::Subscriber output_sub = nh.subscribe("/Controller/DOF/g", 1, &ControlUI::output_val_callback, this);

	//Default use dof_x
	//ros::Subscriber dof_sub = nh.subscribe("a", 1, dof_val_callback);

	//For controls
	ros::Subscriber con_KP_val_sub = nh.subscribe("a", 1, &ControlUI::con_KP_val_callback, this);
	ros::Subscriber con_KI_sub = nh.subscribe("a", 1, &ControlUI::con_KI_val_callback, this);
	ros::Subscriber con_KD_sub = nh.subscribe("a", 1, &ControlUI::con_KD_val_callback, this);
	ros::Subscriber actmin_sub = nh.subscribe("a", 1, &ControlUI::actmin_val_callback, this);
	ros::Subscriber actmax_sub = nh.subscribe("a", 1, &ControlUI::actmax_val_callback, this);


	//Thrusters Subscriber [thrusters.msg]
	ros::Subscriber thruster_sub = nh.subscribe("/thruster_speed", 1, &ControlUI::thruster_val_callback, this);

	//DoFs Subscribers (Depth, Yaw, Pitch, Roll, X , Y) [ControlData.msg]
	ros::Subscriber depth_dof_sub = nh.subscribe("/Controller/DOF/Depth", 1, &ControlUI::depth_dof_callback, this);
	ros::Subscriber yaw_dof_sub = nh.subscribe("/Controller/DOF/Yaw", 1, &ControlUI::yaw_dof_callback, this);
	ros::Subscriber pitch_dof_sub = nh.subscribe("/Controller/DOF/Pitch", 1, &ControlUI::pitch_dof_callback, this);
	ros::Subscriber roll_dof_sub = nh.subscribe("/Controller/DOF/Roll", 1, &ControlUI::roll_dof_callback, this);
	ros::Subscriber x_dof_sub = nh.subscribe("/Controller/DOF/X", 1, &ControlUI::x_dof_callback, this);
	ros::Subscriber y_dof_sub = nh.subscribe("/Controller/DOF/Y", 1, &ControlUI::y_dof_callback, this);

	//Graph controller points
	ros::Subscriber graph_update = nh.subscribe("/controller_points", 1, &ControlUI::controllerPointsCallBack, this);
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
	//Telemetry
	params["setpt_val"] = "3.5";
	params["sensor_val"] = "3.2";
	params["error_val"] = "1.0";
	params["KP_val"] = "2.0";
	params["KI_val"] = "4.5";
	params["KD_val"] = "3.7";
	params["output_val"] = "9.8";

	//Thruster values 
	params["thruster_val_1"] = "1.4";
	params["thruster_val_2"] = "1.4";
	params["thruster_val_3"] = "1.4";
	params["thruster_val_4"] = "1.4";
	params["thruster_val_5"] = "1.4";
	params["thruster_val_6"] = "1.4";
	params["thruster_val_7"] = "1.4";
	params["thruster_val_8"] = "1.4";

	params["dof_comboBox"] = "pos_X";
	params["goal_val"] = "10.3";

	//Advanced
	params["fwd_check"] = "false";
	params["fwd_val"] = "3.2";
	params["depth_check"] = "false";
	params["depth_val"] = "3.5";
	params["yaw_check"] = "false";
	params["yaw_val"] = "4.3";
	params["sm_check"] = "false";
	params["sm_val"] = "4.3";

	//Controls
	params["con_KP_val"] = "4.6";
	params["con_KI_val"] = "7.5";
	params["con_KD_val"] = "2.4";
	params["actmin_val"] = "0.5";
	params["actmax_val"] = "5.7";
		 
}

void ControlUI::initialiseParameters() {
	//Telemetry part
	ui.setpt_val->setText(params.find("setpt_val")->second.c_str());
	ui.sensor_val->setText(params.find("sensor_val")->second.c_str());
	ui.error_val->setText(params.find("error_val")->second.c_str());
	ui.KP_val->setText(params.find("KP_val")->second.c_str());
	ui.KI_val->setText(params.find("KI_val")->second.c_str());
	ui.KD_val->setText(params.find("KD_val")->second.c_str());
	ui.output_val->setText(params.find("output_val")->second.c_str());
	ui.thruster_val_1->setText(params.find("thruster_val_1")->second.c_str());
	ui.thruster_val_2->setText(params.find("thruster_val_2")->second.c_str());
	ui.thruster_val_3->setText(params.find("thruster_val_3")->second.c_str());
	ui.thruster_val_4->setText(params.find("thruster_val_4")->second.c_str());
	ui.thruster_val_5->setText(params.find("thruster_val_5")->second.c_str());
	ui.thruster_val_6->setText(params.find("thruster_val_6")->second.c_str());
	ui.thruster_val_7->setText(params.find("thruster_val_7")->second.c_str());
	ui.thruster_val_8->setText(params.find("thruster_val_8")->second.c_str());

	int index = ui.dof_comboBox->findText(params.find("dof_comboBox")->second.c_str());
	if (index == -1) index=0;
	ui.dof_comboBox->setCurrentIndex(index);
	ui.goal_val->setText(params.find("goal_val")->second.c_str());

	//Advanced part
	if (params.find("fwd_check")->second.compare("true") == 0)
	 	{ ui.fwd_check->setChecked(true); }
	 if (params.find("depth_check")->second.compare("true") == 0)
	 	{ ui.depth_check->setChecked(true); }
	 if (params.find("yaw_check")->second.compare("true") == 0)
	 	{ ui.yaw_check->setChecked(true); }
	 if (params.find("sm_check")->second.compare("true") == 0)
	 	{ ui.sm_check->setChecked(true); }
	ui.fwd_val->setText(params.find("fwd_val")->second.c_str());
	ui.depth_val->setText(params.find("depth_val")->second.c_str());
	ui.yaw_val->setText(params.find("yaw_val")->second.c_str());
	ui.sm_val->setText(params.find("sm_val")->second.c_str());

	//Controls part
	ui.con_KP_val->setText(params.find("con_KP_val")->second.c_str());
	ui.con_KI_val->setText(params.find("con_KI_val")->second.c_str());
	ui.con_KD_val->setText(params.find("con_KD_val")->second.c_str());
	ui.actmin_val->setText(params.find("actmin_val")->second.c_str());
	ui.actmax_val->setText(params.find("actmax_val")->second.c_str());

}

/*
	DoF Subscribers
*/
void ControlUI::depth_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	depth = msg->val;
	depth_kp = msg->kp;
	depth_ki = msg->ki;
	depth_kd = msg->kd;
	dofSelected(5);
}
void ControlUI::yaw_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	yaw = msg->val;
	yaw_kp = msg->kp;
	yaw_ki = msg->ki;
	yaw_kd = msg->kd;
	dofSelected(2);
}
void ControlUI::pitch_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	pitch = msg->val;
	pitch_kp = msg->kp;
	pitch_ki = msg->ki;
	pitch_kd = msg->kd;
	dofSelected(4);
}
void ControlUI::roll_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	roll = msg->val;
	roll_kp = msg->kp;
	roll_ki = msg->ki;
	roll_kd = msg->kd;
	dofSelected(3);
}
void ControlUI::x_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	x = msg->val;
	x_kp = msg->kp;
	x_ki = msg->ki;
	x_kd = msg->kd;
	dofSelected(0);
}
void ControlUI::y_dof_callback(const bbauv_msgs::ControlData::ConstPtr& msg){
	y = msg->val;
	y_kp = msg->kp;
	y_ki = msg->ki;
	y_kd = msg->kd;
	dofSelected(1);
}

void ControlUI::controllerPointsCallBack(const bbauv_msgs::controller data) {
	graphDepthSetPt = data.depth_setpoint;
	graphDepthOut = data.depth_input;
}

//To plot the graph of sensors and setpt
void ControlUI::initializeGraph() {
	//Make legend visible
//	ui.graph_canvas->legend->setVisible(true);
//  ui.graph_canvas->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignRight|Qt::AlignBottom);

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
	QObject::connect(ui.graph_canvas, &QCustomPlot::mouseMove, mouseclicked);

	x_org = 0;

	//Plot the graph
	ui.graph_canvas->replot();
}


/* Functions for the subscribers to subscribe to topics
*/

void ControlUI::setpt_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["setpt_val"] = msg->data;
}
void ControlUI::sensor_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["sensor_val"] = msg->data;
}
void ControlUI::error_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["error_val"] = msg->data;
}
void ControlUI::KP_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["KP_val"] = msg->data;
}
void ControlUI::KI_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["KI_val"] = msg->data;
}
void ControlUI::KD_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["KD_val"] = msg->data;
}
void ControlUI::output_val_callback (const std_msgs::Float32::ConstPtr& msg) {
	params["output_val"] = msg->data;
}

void ControlUI::thruster_val_callback (const bbauv_msgs::thruster::ConstPtr& msg) {
	params["thruster_val_1"] = msg->speed1;
	params["thruster_val_2"] = msg->speed2;
	params["thruster_val_3"] = msg->speed3;
	params["thruster_val_4"] = msg->speed4;
	params["thruster_val_5"] = msg->speed5;
	params["thruster_val_6"] = msg->speed6;
	params["thruster_val_7"] = msg->speed7;
	params["thruster_val_8"] = msg->speed8;
	ui.thruster_val_1->setText(params.find("thruster_val_1")->second.c_str());
	ui.thruster_val_2->setText(params.find("thruster_val_2")->second.c_str());
	ui.thruster_val_3->setText(params.find("thruster_val_3")->second.c_str());
	ui.thruster_val_4->setText(params.find("thruster_val_4")->second.c_str());
	ui.thruster_val_5->setText(params.find("thruster_val_5")->second.c_str());
	ui.thruster_val_6->setText(params.find("thruster_val_6")->second.c_str());
	ui.thruster_val_7->setText(params.find("thruster_val_7")->second.c_str());
	ui.thruster_val_8->setText(params.find("thruster_val_8")->second.c_str());
}

void ControlUI::dof_val_callback(const std_msgs::String::ConstPtr& msg){
	params["dof_comboBox"] = msg->data;
}
 void ControlUI::goal_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["goal_val"] = msg->data;
}
void ControlUI::fwdcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["fwd_check"] = msg->data;
}
void ControlUI::fwd_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["fwd_val"] = msg->data;
}
void ControlUI::depthcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["depth_check"] = msg->data;
}
void ControlUI::depth_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["depth_val"] = msg->data;
}
void ControlUI::yawcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["yaw_check"] = msg->data;
}
void ControlUI::yaw_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["yaw_val"] = msg->data;
}
void ControlUI::smcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["sm_check"] = msg->data;
}
void ControlUI::sm_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["sm_val"] = msg->data;
}

void ControlUI::con_KP_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KP_val"] = msg->data;
}
void ControlUI::con_KI_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KI_val"] = msg->data;
}
void ControlUI::con_KD_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KD_val"] = msg->data;
}
void ControlUI::actmin_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["actmin_val"] = msg->data;
}
void ControlUI::actmax_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["actmax_val"] = msg->data;
}

//////////////////////////
// Non Class functions //
/////////////////////////

void updateGraph() {
	double x_val = (ros::Time::now() - controlUI->startTime).toSec();

//	if (graph_x.) {
//		ui.graph_canvas->graph(0)->removeDataBefore(1);
//		ui.graph_canvas->graph(1)->removeDataBefore(1);
//	}

	controlUI->ui.graph_canvas->graph(0)->addData(x_val, controlUI->graphDepthSetPt);//Set Point
	controlUI->ui.graph_canvas->graph(1)->addData(x_val, controlUI->graphDepthOut);//Output
	controlUI->ui.graph_canvas->graph(1)->rescaleAxes();
	controlUI->ui.graph_canvas->replot();
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
	file << "output_val " << controlUI->ui.output_val->text().toUtf8().constData() << "\n";
	file << "thruster_val " << controlUI->ui.thruster_val_1->text().toUtf8().constData()
	     << controlUI->ui.thruster_val_2->text().toUtf8().constData()
	     << controlUI->ui.thruster_val_3->text().toUtf8().constData()
	     << controlUI->ui.thruster_val_4->text().toUtf8().constData()
	     <<	controlUI->ui.thruster_val_5->text().toUtf8().constData()
	     << controlUI->ui.thruster_val_6->text().toUtf8().constData()
	     <<	controlUI->ui.thruster_val_7->text().toUtf8().constData()
	     << controlUI->ui.thruster_val_8->text().toUtf8().constData() << "\n";


	file << "dof_comboBox " << controlUI->ui.dof_comboBox->currentText().toUtf8().constData() << "\n";
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

void openFile() {
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
		while (getline(file, line)){
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
void mouseclicked(QMouseEvent *event) {
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
		float temp;
		//oss << std::fixed << std::setprecision(3);
		std_msgs::Float32 msg;
		ros::Publisher yaw_val_pub = nh.advertise<std_msgs::Float32>("yaw_val_pub", 1);
		if (controlUI->ui.yaw_check->isChecked()){
			temp = atof(controlUI->params.find("yaw_val")->second.c_str());
			goal.heading_setpoint= temp;
		} else {
			temp = 0.0;
			goal.heading_setpoint=temp;
		}
		msg.data = temp;
		yaw_val_pub.publish(msg);

		ros::Publisher fwd_val_pub = nh.advertise<std_msgs::Float32>("fwd_val_pub", 1);
		if (controlUI->ui.fwd_check->isChecked()){
			temp = atof(controlUI->params.find("fwd_val")->second.c_str());
			goal.forward_setpoint=temp;
		} else {
			temp = 0.0;
			goal.forward_setpoint = temp;
		}
		msg.data = temp;
		fwd_val_pub.publish(msg);

		ros::Publisher depth_val_pub = nh.advertise<std_msgs::Float32>("depth_val_pub", 1);
		if (controlUI->ui.depth_check->isChecked()){
			temp = atof(controlUI->params.find("depth_val")->second.c_str());
			goal.depth_setpoint=temp;
		} else {
			temp = 0.0;
			goal.depth_setpoint = temp; 
		}
		msg.data = temp;
		depth_val_pub.publish(msg);

		ros::Publisher sm_val_pub = nh.advertise<std_msgs::Float32>("sm_val_pub", 1);
		if (controlUI->ui.sm_check->isChecked()){
			temp = atof(controlUI->params.find("sm_val")->second.c_str());
			goal.sidemove_setpoint=temp;
		} else {
			temp = 0.0;
			goal.sidemove_setpoint = temp; 
		}
		msg.data = temp;
		sm_val_pub.publish(msg);
		
		ac.sendGoal(goal);
		ros::spinOnce();

		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
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
		ros::NodeHandle nh;
		float temp;
		std_msgs::Float32 msg;

		ros::Publisher con_KP_pub = nh.advertise<std_msgs::Float32>("con_KP_pub", 1);
		temp = atof(controlUI->params.find("con_KP_val")->second.c_str());
		msg.data = temp;
		con_KP_pub.publish(msg);

		ros::Publisher con_KD_pub = nh.advertise<std_msgs::Float32>("con_KD_pub", 1);
		temp = atof(controlUI->params.find("con_KD_val")->second.c_str());
		msg.data = temp;
		con_KD_pub.publish(msg);

		ros::Publisher con_KI_pub = nh.advertise<std_msgs::Float32>("con_KI_pub", 1);
		temp = atof(controlUI->params.find("con_KI_val")->second.c_str());
		msg.data = temp;
		con_KI_pub.publish(msg);

		ros::Publisher actmin_pub = nh.advertise<std_msgs::Float32>("actmin_pub", 1);
		temp = atof(controlUI->params.find("actmin_val")->second.c_str());
		msg.data = temp;
		actmin_pub.publish(msg);

		ros::Publisher actmax_pub = nh.advertise<std_msgs::Float32>("actmax_pub", 1);
		temp = atof(controlUI->params.find("actmax_val")->second.c_str());
		msg.data = temp;
		actmax_pub.publish(msg);
	}
}

/*
	Updates DoF values in the UI
*/
void dofSelected(int index){
	ROS_INFO("Current selected index in DoF: %i", index);
	switch(index){
		//dof x
		case 0:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->x).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->x_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->x_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->x_kd).c_str());
			break;

		//dof y
		case 1:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->y).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->y_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->y_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->y_kd).c_str());
		break;

		//yaw
		case 2:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->yaw).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->yaw_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->yaw_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->yaw_kd).c_str());
		break;

		//roll
		case 3:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->roll).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->roll_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->roll_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->roll_kd).c_str());
		break;

		//pitch
		case 4:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->pitch).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->pitch_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->pitch_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->pitch_kd).c_str());
		break;

		//depth
		case 5:
			controlUI->ui.sensor_val->setText(boost::lexical_cast<std::string>(controlUI->depth).c_str());
			controlUI->ui.KP_val->setText(boost::lexical_cast<std::string>(controlUI->depth_kp).c_str());
			controlUI->ui.KI_val->setText(boost::lexical_cast<std::string>(controlUI->depth_ki).c_str());
			controlUI->ui.KD_val->setText(boost::lexical_cast<std::string>(controlUI->depth_kd).c_str());
		break;
	}
}
