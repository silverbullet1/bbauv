#include "controlui_add.h"

static string parameters[23];

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");
	int i;
	for (i=0; i<23; i++) { parameters[i] = "0"; }

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
	QObject::connect(ui.disabledButton, &QAbstractButton::released, disableButton);

	window->show();

	return app.exec();
}

static void initialize(){
	ros::NodeHandle node;
	//Subscribe to the stuff and initialize them: to find out the nodes 

	initialiseParameters();
	//Suppose to set text to the subscribed data if not initialise default
	
}

static string getdate(){
	int max_date = 12;
	time_t now;
	char date[max_date];
	date[0] = '\0';
	now = time(NULL);
	if (now != -1) { strftime(date, max_date, "%d%m%y", gmtime(&now)); }
	return date;
}

static void initialiseParameters(){
	//Telemetry part
	ui.setpt_val->setText(parameters[0].c_str());
	ui.sensor_val->setText(parameters[1].c_str());
	ui.error_val->setText(parameters[2].c_str());
	ui.KP_val->setText(parameters[3].c_str());
	ui.KI_val->setText(parameters[4].c_str());
	ui.KD_val->setText(parameters[5].c_str());
	ui.output_val->setText(parameters[6].c_str());
	ui.thruster_val->setText(parameters[7].c_str());

	int index = ui.dof_comboBox->findText(parameters[8].c_str());
	if (index == -1) index=0;
	ui.dof_comboBox->setCurrentIndex(index);
	ui.goal_val->setText(parameters[9].c_str());

	//Advanced part
	//Checkboxes
	if (parameters[10].compare("true") == 0)
		{ ui.fwd_check->setChecked(true); }
	else { ui.fwd_check->setChecked(false); }
	ui.fwd_val->setText(parameters[11].c_str());

	if (parameters[12].compare("true") == 0)
		{ ui.depth_check->setChecked(true); }
	else { ui.depth_check->setChecked(false); }
	ui.depth_val->setText(parameters[13].c_str());

	if (parameters[14].compare("true") == 0)
		{ ui.yaw_check->setChecked(true); }
	else { ui.yaw_check->setChecked(false); }
	ui.yaw_val->setText(parameters[15].c_str());


	if (parameters[16].compare("true") == 0)
		{ ui.sm_check->setChecked(true); }
	else { ui.sm_check->setChecked(false); }
	ui.sm_val->setText(parameters[17].c_str());

	//Control parameters
	ui.con_KP_val->setText(parameters[18].c_str());
	ui.con_KI_val->setText(parameters[19].c_str());
	ui.con_KD_val->setText(parameters[20].c_str());
	ui.actmin_val->setText(parameters[21].c_str());
	ui.actmax_val->setText(parameters[22].c_str());
}

static void saveFile(){
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
	file << "thruster_val " << ui.thruster_val->text().toUtf8().constData() << "\n";

	file << "dof_comboBox " << ui.dof_comboBox->currentText().toUtf8().constData() << "\n";
	file << "goal_val " << ui.goal_val->text().toUtf8().constData() << "\n";

	if ( ui.fwd_check->isChecked() ){ 
		file << "fwd_val " << "true " << ui.fwd_val->text().toUtf8().constData() << "\n";
	}
	else {
		file << "fwd_val " << "false " << ui.fwd_val->text().toUtf8().constData() << "\n";
	}

	if ( ui.depth_check->isChecked() ){ 
		file << "depth_val " << "true " << ui.depth_val->text().toUtf8().constData() << "\n";
	}
	else {
		file << "depth_val " << "false " << ui.depth_val->text().toUtf8().constData() << "\n";
	}

	if ( ui.yaw_check->isChecked() ){ 
		file << "yaw_val " << "true " << ui.yaw_val->text().toUtf8().constData() << "\n";
	}
	else {
		file << "yaw_val " << "false " << ui.yaw_val->text().toUtf8().constData() << "\n";
	}

	if ( ui.sm_check->isChecked() ){ 
		file << "sm_val " << "true " << ui.sm_val->text().toUtf8().constData() << "\n";
	}
	else {
		file << "sm_val " << "false " << ui.sm_val->text().toUtf8().constData() << "\n";
	}

	file << "con_KP_val " << ui.con_KP_val->text().toUtf8().constData() << "\n";
	file << "con_KI_val " << ui.con_KI_val->text().toUtf8().constData() << "\n";
	file << "con_KD_val " << ui.con_KD_val->text().toUtf8().constData()<< "\n";
	file << "actmin_val " << ui.actmin_val->text().toUtf8().constData() << "\n";
	file << "actmax_val " << ui.actmax_val->text().toUtf8().constData()<< "\n";

	file.close();
}

static void openFile(){
	string line;
	vector<string> tokens;
	//string arr[23];
	int i=0, j=0;
	QString selfilter = QString("*.bag");
	QString filename = QFileDialog::getOpenFileName(window, QString("Open controls file"), QDir::currentPath(), 
	QString(".txt files (*.txt);; All files (*.*)"), &selfilter);

	string filename_string = filename.toUtf8().constData();
	ifstream file;
	file.open(filename_string.c_str());
	if (file.is_open()){
		while (getline(file, line)){
			boost::algorithm::split(tokens, line, boost::algorithm::is_any_of(" "));
			for(i=0; i<tokens.size(); i++)
				if (i != 0){
					parameters[j] = tokens[i];
					cout << tokens[i] << endl;
					j++;
				}
			}
		initialiseParameters();
		file.close();
		}
		
	else {
		QMessageBox::critical(ui.centralwidget, "Error", "Could not open file");
		return;
	} 	
}

static void enableButton(){

}

static void disableButton(){

}

//To connect to actionlib and move the robot
static void fire(){

}