#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>

#include "controlui.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>

static Ui::ControlSysUI ui;
static QMainWindow *window;

int main(int argc, char **argv) {
	ros::init(argc, argv, "controlui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);

	window->show();

	return app.exec();
}