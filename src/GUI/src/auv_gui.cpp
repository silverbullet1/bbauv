#include <QApplication>
#include <QFileDialog>
#include "auv_gui.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cstdlib>
#include <string.h>

static Ui::Vision ui;

static void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

static void selectBottomFilter(int selectedIndex){
	ui.bottomfilter->setItemText(selectedIndex,"Hello");
}

static void openFile(bool open){
	//Apparently QMainWIndow needs to be casted to a QWidget? 

	QWidget *widget = new QWidget;
	QString selfilter = QString("BAG(*.bag)");
	QString filename = QFileDialog::getOpenFileName(ui.centralwidget, QString("Open bag file"), QDir::currentPath(), 
	 	QString("BAG files (*.bag);; All files (*.*)"), &selfilter);
	
	// if (!filename.isNull()) { qDebug.toAscii(); }
	// //Try to run the bag file from a new terminal in rosrun
	// char command[500];
	// strcpy(command, "gnome-terminal -e 'bash -c " + "\"" + "rosbag play "  + filename + "; exec bash\"" + "'");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auv_gui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	QMainWindow *window = new QMainWindow;
	ui.setupUi(window);
	
	QObject::connect(ui.bottomfilter, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), selectBottomFilter);
	QObject::connect(ui.actionOpen, &QAction::triggered, openFile);

	window->show();


	ros::NodeHandle node;
	//Subscribe to a topic "chatter", with queue size 10 and callback chatterCallback
	ros::Subscriber sub = node.subscribe("chatter", 10, chatterCallback);
	return app.exec();
}

