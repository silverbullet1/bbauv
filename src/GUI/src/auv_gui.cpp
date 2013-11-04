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
	//Each filter selected should show different image processing
}

static void openFile(bool open){
	//Apparently QMainWIndow needs to be casted to a QWidget? 
	//QFileDialog to open file
	QWidget *widget = new QWidget;
	QString selfilter = QString("BAG(*.bag)");
	// QString filename = QFileDialog::getOpenFileName(static_cast<QWidget *>(ui), QString("Open bag file"), QDir::currentPath(), 
	//  	QString("BAG files (*.bag);; All files (*.*)"), &selfilter);
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
	QObject::connect(ui.actionOpen, static_cast<void(QAction::*)(bool)>(&QAction::triggered), openFile);

	//Set image in QFrame (testing!)
	QPixmap pixmap;
	pixmap.load("penguin.jpeg");
	QPalette* palette = new QPalette();
	QBrush* brush = new QBrush(pixmap);
	//QVBoxLayout* frontcam_layout = new QVBoxLayout();
	palette->setBrush(QPalette::Background, *brush);
	ui.frontcam_2->setPalette(*palette);

	window->show();


	ros::NodeHandle node;
	//Subscribe to a topic "chatter", with queue size 10 and callback chatterCallback
	ros::Subscriber sub = node.subscribe("chatter", 10, chatterCallback);
	ros::spin();

	return app.exec();
}

