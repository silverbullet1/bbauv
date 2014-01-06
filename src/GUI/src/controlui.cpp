#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLineEdit>

#include "controlui.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>

using namespace std;

static Ui::ControlSysUI ui;
static QMainWindow *window;

void initialize(){
	ros::NodeHandle node;
	//Subscribe to the stuff and initialize them

	//Telemetry part
	ui.setpt_val->setText("5");
	ui.sensor_val->setText("5");
	ui.error_val->setText("5");
	ui.KP_val->setText("5");
	ui.KI_val->setText("5");
	ui.KD_val->setText("5");
	ui.output_val->setText("5");
	ui.thruster_val->setText("5");

	ui.goal_val->setText("5");

	//Advanced part
	//Checkboxes
	ui.fwd_check->setChecked(true);
	ui.depth_check->setChecked(true);
	ui.yaw_check->setChecked(true);
	ui.sm_check->setChecked(true);
	ui.fwd_val->setText("5");
	ui.depth_val->setText("5");
	ui.yaw_val->setText("5");
	ui.sm_val->setText("5");

	//Control parameters
	ui.con_KP_val->setText("5");
	ui.con_KI_val->setText("5");
	ui.con_KD_val->setText("5");
	ui.actmin_val->setText("5");
	ui.actmax_val->setText("5");
}

static void saveFile(){
	//Open a .txt file
	string filename = 
	ofstream file (filename);

	cout << ui.error_val->text().toUtf8().constData() << endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);
	window->setFixedSize(window->geometry().width(), window->geometry().height());

	initialize();

	QObject::connect(ui.actionSave, &QAction::triggered, saveFile);
	QObject::connect(ui.saveButton, &QAbstractButton::released, saveFile);

	window->show();

	return app.exec();
}