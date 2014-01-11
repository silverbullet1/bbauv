/* 
	controlui.cpp
	A GUI for tuning the control systems
	Date created: 8 Jan 2014
	Author: Lynnette Ng
*/

#include "controlui_add.h"
#include "qcustomplot.h"


static map <string, string> params; //Map for parameters
static QVector<double> graph_x(101), graph_setpt(101), graph_output(101); //Vector values for graph
static bool live=false;					//Boolean whether UI is connected to robot
static bool enable=false;

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("live", live, bool(false));
	if (!live){
		ROS_INFO("%s", "Not going live");
		initialiseDefault();
	}
	else {
		ROS_INFO("%s", "Going live!");
		subscribeToData();
	}

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);
	window->setFixedSize(window->geometry().width(), window->geometry().height());

	initialiseParameters();

	QObject::connect(ui.actionSave, &QAction::triggered, saveFile);
	QObject::connect(ui.fireButton, &QAbstractButton::released, fire);
	QObject::connect(ui.actionOpen, &QAction::triggered, openFile);
	QObject::connect(ui.enabledButton, &QAbstractButton::released, enableButton);
	QObject::connect(ui.tuneButton, &QAbstractButton::released, tuneButton);
	QObject::connect(ui.sendButton, &QAbstractButton::released, sendButton);
	QObject::connect(ui.dof_comboBox,
					 static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
					 dofSelected);

	graph_test();

	window->show();

	return app.exec();
}

string getdate(){
	int max_date = 12;
	time_t now;
	char date[max_date];
	date[0] = '\0';
	now = time(NULL);
	if (now != -1) { strftime(date, max_date, "%d%m%y", gmtime(&now)); }
	return date;
}

void initialiseDefault(){
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

	//Graph: initialise random function
	for (int i=0; i<101; i++){
	  graph_x[i] = i-5; // x goes from -1 to 1
	  graph_setpt[i] = 2; //straight line function
	}
	int x_val;
	for (int i=0; i<101; i++){
	  x_val = i/50.0 - 1; // x goes from -1 to 1
	  graph_output[i] = x_val*x_val; // exponential function
	}
		 
}

void initialiseParameters(){
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

void saveFile(){
	//Open a .txt file
	string filename = "controls";
	filename.append("_");
	filename.append(getdate());
	filename.append(".txt");
	ofstream file;
	file.open(filename.c_str());
	file << "setpt_val " << ui.setpt_val->text().toUtf8().constData() << "\n";
	file << "sensor_val " << ui.sensor_val->text().toUtf8().constData() << "\n";
	file << "error_val " << ui.error_val->text().toUtf8().constData()<< "\n";
	file << "KP_val " << ui.KP_val->text().toUtf8().constData() << "\n";
	file << "KI_val " << ui.KI_val->text().toUtf8().constData() << "\n";
	file << "KD_val " << ui.KD_val->text().toUtf8().constData()<< "\n";
	file << "output_val " << ui.output_val->text().toUtf8().constData() << "\n";
	file << "thruster_val " << ui.thruster_val_1->text().toUtf8().constData() << ui.thruster_val_2->text().toUtf8().constData() 
	<< ui.thruster_val_3->text().toUtf8().constData() << ui.thruster_val_4->text().toUtf8().constData() <<
	ui.thruster_val_5->text().toUtf8().constData() << ui.thruster_val_6->text().toUtf8().constData() << 
	ui.thruster_val_7->text().toUtf8().constData() << ui.thruster_val_8->text().toUtf8().constData()<< "\n";


	file << "dof_comboBox " << ui.dof_comboBox->currentText().toUtf8().constData() << "\n";
	file << "goal_val " << ui.goal_val->text().toUtf8().constData() << "\n";

	if ( ui.fwd_check->isChecked() ){ 
		file << "fwd_check " << "true" << "\n";
	}
	else {
		file << "fwd_check " << "false " << "\n";
	}
	file << "fwd_val "  << ui.fwd_val->text().toUtf8().constData() << "\n";

	if ( ui.depth_check->isChecked() ){ 
		file << "depth_check " << "true" << "\n";
	}
	else {
		file << "depth_check " << "false " << "\n";
	}
	file << "depth_val "  << ui.depth_val->text().toUtf8().constData() << "\n";

	if ( ui.yaw_check->isChecked() ){ 
		file << "yaw_check " << "true" << "\n";
	}
	else {
		file << "yaw_check " << "false " << "\n";
	}
	file << "yaw_val "  << ui.yaw_val->text().toUtf8().constData() << "\n";

	if ( ui.sm_check->isChecked() ){ 
		file << "sm_check " << "true" << "\n";
	}
	else {
		file << "sm_check " << "false " << "\n";
	}
	file << "sm_val "  << ui.sm_val->text().toUtf8().constData() << "\n";

	file << "con_KP_val " << ui.con_KP_val->text().toUtf8().constData() << "\n";
	file << "con_KI_val " << ui.con_KI_val->text().toUtf8().constData() << "\n";
	file << "con_KD_val " << ui.con_KD_val->text().toUtf8().constData()<< "\n";
	file << "actmin_val " << ui.actmin_val->text().toUtf8().constData() << "\n";
	file << "actmax_val " << ui.actmax_val->text().toUtf8().constData()<< "\n";

	file.close();
	QMessageBox::information(ui.centralwidget, "File saved", "File successfully saved! :)");
}

void openFile(){
	string line;
	vector<string> tokens;
	QString selfilter = QString("*.bag");
	QString filename = QFileDialog::getOpenFileName(window, QString("Open controls file"), QDir::currentPath(), 
	QString(".txt files (*.txt);; All files (*.*)"), &selfilter);

	string filename_string = filename.toUtf8().constData();
	ifstream file;
	file.open(filename_string.c_str());
	if (file.is_open()){
		string msg = "Loading file " + filename_string + "...";
		ROS_INFO("%s", msg.c_str());
		while (getline(file, line)){
			boost::algorithm::split(tokens, line, boost::algorithm::is_any_of(" "));
			params[tokens[0]] = tokens[1];
			}
		initialiseParameters();
		file.close();
		}
	else {
		QMessageBox::critical(ui.centralwidget, "Error", "Could not open file");
		return;
	} 	
}

//To subscribe to data topics: currently under default!! 
void subscribeToData(){
	ros::NodeHandle nh;
	//For telemetry
	ros::Subscriber setpt_val_sub = nh.subscribe("a", 1, setpt_val_callback);
	ros::Subscriber sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
	ros::Subscriber error_sub = nh.subscribe("a", 1, error_val_callback);
	ros::Subscriber KP_val_sub = nh.subscribe("a", 1, KP_val_callback);
	ros::Subscriber KI_sub = nh.subscribe("a", 1, KI_val_callback);
	ros::Subscriber KD_sub = nh.subscribe("a", 1, KD_val_callback);
	ros::Subscriber output_sub = nh.subscribe("a", 1, output_val_callback);
	
	//Initialise the 8 thrusters -- they're all in one topic?
	ros::Subscriber thruster1_sub = nh.subscribe("/thruster_speed", 1, thruster_val_1_callback);
	ros::Subscriber thruster2_sub = nh.subscribe("/thruster_speed", 1, thruster_val_2_callback);
	ros::Subscriber thruster3_sub = nh.subscribe("/thruster_speed", 1, thruster_val_3_callback);
	ros::Subscriber thruster4_sub = nh.subscribe("/thruster_speed", 1, thruster_val_4_callback);
	ros::Subscriber thruster5_sub = nh.subscribe("/thruster_speed", 1, thruster_val_5_callback);
	ros::Subscriber thruster6_sub = nh.subscribe("/thruster_speed", 1, thruster_val_6_callback);
	ros::Subscriber thruster7_sub = nh.subscribe("/thruster_speed", 1, thruster_val_7_callback);
	ros::Subscriber thruster8_sub = nh.subscribe("/thruster_speed", 1, thruster_val_8_callback);

	//Default use dof_x
	//ros::Subscriber dof_sub = nh.subscribe("a", 1, dof_val_callback);
	ros::Subscriber goal_val_sub = nh.subscribe("a", 1, goal_val_callback);

	//For advanced
	ros::Subscriber fwdcheck_sub = nh.subscribe("a", 1, fwdcheck_callback);
	ros::Subscriber depthcheck_sub = nh.subscribe("a", 1, depthcheck_callback);
	ros::Subscriber yawcheck_sub = nh.subscribe("a", 1, yawcheck_callback);
	ros::Subscriber smcheck_sub = nh.subscribe("a", 1, smcheck_callback);
	ros::Subscriber fwd_val_sub = nh.subscribe("a", 1, fwd_val_callback);
	ros::Subscriber depth_val_sub = nh.subscribe("/depth", 1, depth_val_callback);
	ros::Subscriber yaw_val_sub = nh.subscribe("a", 1, yaw_val_callback);
	ros::Subscriber sm_val_sub = nh.subscribe("a", 1, sm_val_callback);

	//For controls
	ros::Subscriber con_KP_val_sub = nh.subscribe("a", 1, con_KP_val_callback);
	ros::Subscriber con_KI_sub = nh.subscribe("a", 1, con_KI_val_callback);
	ros::Subscriber con_KD_sub = nh.subscribe("a", 1, con_KD_val_callback);
	ros::Subscriber actmin_sub = nh.subscribe("a", 1, actmin_val_callback);
	ros::Subscriber actmax_sub = nh.subscribe("a", 1, actmax_val_callback);

	//For graph
	ros::Subscriber graph_setpt_sub = nh.subscribe("a", 1, graph_setpt_callback);
	ros::Subscriber graph_output_sub = nh.subscribe("a", 1, graph_output_callback);
}

//To enable all the check boxes
void enableButton(){
	enable = !enable;
	if (enable){
		ui.fwd_check->setChecked(true);
		ui.depth_check->setChecked(true);
		ui.yaw_check->setChecked(true);
		ui.sm_check->setChecked(true);
	}
	else {
		ui.fwd_check->setChecked(false);
		ui.depth_check->setChecked(false);
		ui.yaw_check->setChecked(false);
		ui.sm_check->setChecked(false);
	}
}

//To send goal value
void fire(){
	if (!live){
		QMessageBox::information(ui.centralwidget, "Fire!", "Bang! Boom! Bam!");
	}
	else {
		ros::NodeHandle nh;
		float temp;
		istringstream iss("");
		std_msgs::Float32 msg;
		ros::Publisher goal_pub = nh.advertise<std_msgs::Float32>("goal_pub", 1);
		ros::Rate loop_rate(10);
		iss.str(params.find("goal_val")->second);
		iss >> temp;
		msg.data = temp;
		goal_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//To plot the graph of sensors and setpt
void graph_test() {
	//Make legend visible
	ui.graph_canvas->legend->setVisible(true);
	ui.graph_canvas->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignRight|Qt::AlignBottom);
	//Add the graphs
	ui.graph_canvas->addGraph(ui.graph_canvas->xAxis, ui.graph_canvas->yAxis);
	ui.graph_canvas->graph(0)->setData(graph_x, graph_setpt);
	ui.graph_canvas->graph(0)->setName("Setpt");
	ui.graph_canvas->graph(0)->setPen(QPen(Qt::blue));
	ui.graph_canvas->addGraph();
	//ui.graph_canvas->addGraph(ui.graph_canvas->xAxis2, ui.graph_canvas->yAxis2);
	ui.graph_canvas->graph(1)->setData(graph_x, graph_output);
	ui.graph_canvas->graph(1)->setName("Output (dof)");
	ui.graph_canvas->graph(1)->setPen(QPen(Qt::red));

	// give the axes some labels:
	ui.graph_canvas->xAxis->setLabel("Time (s)");
	ui.graph_canvas->yAxis->setLabel("Output");
	
	// center the axes
	ui.graph_canvas->xAxis->scaleRange(-3.0,ui.graph_canvas->xAxis->range().center());
	ui.graph_canvas->yAxis->scaleRange(-3.0,ui.graph_canvas->yAxis->range().center());
	// if (ui.graph_canvas->yAxis->range().size() < 1.234)
 //  		ui.graph_canvas->yAxis->setRange(ui.graph_canvas->yAxis->range().center(), 0.234, Qt::AlignCenter);
	// if (ui.graph_canvas->xAxis->range().size() < 1.234)
 //  		ui.graph_canvas->xAxis->setRange(ui.graph_canvas->xAxis->range().center(), -1.234, Qt::AlignCenter);

	//For user interaction
	ui.graph_canvas->setInteraction(QCP::iRangeDrag, true);
	ui.graph_canvas->setInteraction(QCP::iRangeZoom, true);
	QObject::connect(ui.graph_canvas, &QCustomPlot::mouseMove, mouseclicked);
	//Plot the graph
	ui.graph_canvas->replot();
}

//Mouse clicked on graph so display data point
void mouseclicked(QMouseEvent *event) {
	ostringstream oss;
	oss << std::fixed << std::setprecision(3);

	double x = ui.graph_canvas->xAxis->pixelToCoord(event->x());
 	double y = ui.graph_canvas->yAxis->pixelToCoord(event->y());

 	oss << "Graph values: x: " << x << "\ty: " << y ;
 	ui.graphvalues->setText(QString::fromStdString(oss.str()));
}

// Send the advanced parameters 
void sendButton(){
	if (!live){
		QMessageBox::information(ui.centralwidget, "Send!", "Bang! Boom! Bam!");
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
		istringstream iss("");
		//oss << std::fixed << std::setprecision(3);
		std_msgs::Float32 msg;
		ros::Publisher yaw_val_pub = nh.advertise<std_msgs::Float32>("yaw_val_pub", 1);
		ros::Rate loop_rate(10);
		if (ui.yaw_check->isChecked()){
			iss.clear();
			iss.str(params.find("yaw_val")->second);
			iss >> temp;
			goal.heading_setpoint= temp;
		}
		else { 
			temp = 0.0;
			goal.heading_setpoint=temp;
		}
		msg.data = temp;
		yaw_val_pub.publish(msg);

		ros::Publisher fwd_val_pub = nh.advertise<std_msgs::Float32>("fwd_val_pub", 1);
		if (ui.fwd_check->isChecked()){
			iss.clear();
			iss.str(params.find("fwd_val")->second);
			iss >> temp;
			goal.forward_setpoint=temp;
		}
		else { 
			temp = 0.0;
			goal.forward_setpoint = temp;
		}
		msg.data = temp;
		fwd_val_pub.publish(msg);

		ros::Publisher depth_val_pub = nh.advertise<std_msgs::Float32>("depth_val_pub", 1);
		if (ui.depth_check->isChecked()){
			iss.clear();
			iss.str(params.find("depth_val")->second);
			iss >> temp;
			goal.depth_setpoint=temp;
		}
		else { 
			temp = 0.0;
			goal.depth_setpoint = temp; 
		}
		msg.data = temp;
		depth_val_pub.publish(msg);

		ros::Publisher sm_val_pub = nh.advertise<std_msgs::Float32>("sm_val_pub", 1);
		if (ui.sm_check->isChecked()){
			iss.clear();
			iss.str(params.find("sm_val")->second);
			iss >> temp;
			goal.sidemove_setpoint=temp;
		}
		else { 
			temp = 0.0;
			goal.sidemove_setpoint = temp; 
		}
		msg.data = temp;
		sm_val_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		ac.sendGoal(goal);

		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
		if (finished_before_timeout){
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
			QMessageBox::information(ui.centralwidget, "Goal status", state.toString().c_str());
		}
		else {
			ROS_INFO("Timeout! Goal not achieved");
			QMessageBox::critical(ui.centralwidget, "Goal status", "Timeout! Goal not achieved");
		}
	}
}

// Tune the control parameters by publishing them
void tuneButton(){
	if (!live){
		QMessageBox::information(ui.centralwidget, "Tune!", "Twinkle Twinkle Little Star~");
	}
	else {
		ros::NodeHandle nh;
		float temp;
		istringstream iss("");
		std_msgs::Float32 msg;
		ros::Rate loop_rate(10);

		ros::Publisher con_KP_pub = nh.advertise<std_msgs::Float32>("con_KP_pub", 1);
		iss.str(params.find("con_KP_val")->second);
		iss >> temp;
		msg.data = temp;
		con_KP_pub.publish(msg);

		ros::Publisher con_KD_pub = nh.advertise<std_msgs::Float32>("con_KD_pub", 1);
		iss.clear();
		iss.str(params.find("con_KD_val")->second);
		iss >> temp;
		msg.data = temp;
		con_KD_pub.publish(msg);

		ros::Publisher con_KI_pub = nh.advertise<std_msgs::Float32>("con_KI_pub", 1);
		iss.clear();
		iss.str(params.find("con_KI_val")->second);
		iss >> temp;
		msg.data = temp;
		con_KI_pub.publish(msg);

		ros::Publisher actmin_pub = nh.advertise<std_msgs::Float32>("actmin_pub", 1);
		iss.clear();
		iss.str(params.find("actmin_val")->second);
		iss >> temp;
		msg.data = temp;
		actmin_pub.publish(msg);

		ros::Publisher actmax_pub = nh.advertise<std_msgs::Float32>("actmax_pub", 1);
		iss.clear();
		iss.str(params.find("actmax_val")->second);
		iss >> temp;
		msg.data = temp;
		actmax_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

// Receive the respective parameters for the dof 
void dofSelected(int index){
	ros::NodeHandle nh;
	ros::Subscriber dof_setpt_sub, sensor_sub, error_sub, output_sub, KP_sub, KI_sub, KD_sub;
	ros::Subscriber thruster_val_1_sub, thruster_val_2_sub, thruster_val_3_sub, thruster_val_4_sub, thruster_val_5_sub, thruster_val_6_sub, thruster_val_7_sub, thruster_val_8_sub;


	switch(index){
		//dof x
		case 1:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		//dof y
		case 2:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		//yaw
		case 3:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		// //roll
		case 4:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		//pitch
		case 5:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		// //depth
		case 6:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);

		break;

		//dof x
		default:
		dof_setpt_sub = nh.subscribe("a", 100, setpt_val_callback);
		sensor_sub = nh.subscribe("a", 1, sensor_val_callback);
		error_sub = nh.subscribe("a", 1, error_val_callback);
		output_sub = nh.subscribe("a", 1, output_val_callback);
		KP_sub = nh.subscribe("a", 1, KP_val_callback);
		KI_sub = nh.subscribe("a", 1, KI_val_callback);
		KD_sub = nh.subscribe("a", 1, KD_val_callback);

		// thruster_val_1_sub = nh.subscribe("a", 1, thruster_val_1_callback);
		// thruster_val_2_sub = nh.subscribe("a", 1, thruster_val_2_callback);
		// thruster_val_3_sub = nh.subscribe("a", 1, thruster_val_3_callback);
		// thruster_val_4_sub = nh.subscribe("a", 1, thruster_val_4_callback);
		// thruster_val_5_sub = nh.subscribe("a", 1, thruster_val_5_callback);
		// thruster_val_6_sub = nh.subscribe("a", 1, thruster_val_6_callback);
		// thruster_val_7_sub = nh.subscribe("a", 1, thruster_val_7_callback);
		// thruster_val_8_sub = nh.subscribe("a", 1, thruster_val_8_callback);
		break;
	}
}


/* Functions for the subscribers to subscribe to topics 
*/

void setpt_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["setpt_val"] = msg->data;
}
void sensor_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["sensor_val"] = msg->data;
}
void error_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["error_val"] = msg->data;
}
void KP_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["KP_val"] = msg->data;
}
void KI_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["KI_val"] = msg->data;
}
void KD_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["KD_val"] = msg->data;
}
void output_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["output_val"] = msg->data;
}

void thruster_val_1_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_1"] = msg->data;
}
void thruster_val_2_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_2"] = msg->data;
}
void thruster_val_3_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_3"] = msg->data;
}
void thruster_val_4_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_4"] = msg->data;
}
void thruster_val_5_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_5"] = msg->data;
}
void thruster_val_6_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_6"] = msg->data;
}
void thruster_val_7_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_7"] = msg->data;
}
void thruster_val_8_callback(const std_msgs::Float32::ConstPtr& msg){
	params["thruster_val_8"] = msg->data;
}

void dof_val_callback(const std_msgs::String::ConstPtr& msg){
	params["dof_comboBox"] = msg->data;
}
 void goal_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["goal_val"] = msg->data;
}
void fwdcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["fwd_check"] = msg->data;
}
void fwd_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["fwd_val"] = msg->data;
}
void depthcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["depth_check"] = msg->data;
}
void depth_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["depth_val"] = msg->data;
}
void yawcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["yaw_check"] = msg->data;
}
void yaw_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["yaw_val"] = msg->data;
}
void smcheck_callback(const std_msgs::Bool::ConstPtr& msg){
	params["sm_check"] = msg->data;
}
void sm_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["sm_val"] = msg->data;
}

void con_KP_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KP_val"] = msg->data;
}
void con_KI_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KI_val"] = msg->data;
}
void con_KD_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["con_KD_val"] = msg->data;
}
void actmin_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["actmin_val"] = msg->data;
}
void actmax_val_callback(const std_msgs::Float32::ConstPtr& msg){
	params["actmax_val"] = msg->data;
}
void graph_setpt_callback(const std_msgs::Float32::ConstPtr& msg){
	graph_setpt.append(msg->data);
}
void graph_output_callback(const std_msgs::Float32::ConstPtr& msg){
	graph_output.append(msg->data);
}
