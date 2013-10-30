#include <QApplication>
#include "auv_gui.h"
#include "ros/ros.h"
#include "mywindow.h"
#include "std_msgs/String.h"

Ui::Vision ui;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void openFIle(int a){
	ui.bottomfilter->setItemText(a,"Hello");
	exit(2);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auv_gui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	Ui::Vision ui;
	QMainWindow *window = new MainWindow;
	ui.setupUi(window);
	//connect(ui.bottomfilter, SIGNAL(textChanged(QString)), this, SLOT(*openFile));
	//QObject::connect(ui.bottomfilter, SIGNAL(currentIndexChanged(int)), ui.bottomfilter, SLOT(openFile(int)));
	window->show();

	
	ros::NodeHandle node;
	//Subscribe to a topic "chatter", with queue size 10 and callback chatterCallback
	ros::Subscriber sub = node.subscribe("chatter", 10, chatterCallback);	
	return app.exec();
}

