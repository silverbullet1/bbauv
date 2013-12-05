#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>

#include "auv_gui.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>

//Macro to change subscriber's topic during run time
#define CHANGE_TOPIC(sub, topic, callback)	\
  (sub = it->subscribe(topic, 1, callback))

static Ui::Vision ui;
static QMainWindow *window;
static image_transport::Subscriber sub1, sub2;
//global ImageTransport to change subscribed topic
static image_transport::ImageTransport *it;

static void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

static void selectBottomFilter(int selectedIndex) {
	ui.bottomfilter->setItemText(selectedIndex, "Hello");
}

static void openFile(bool open){
	QString selfilter = QString("BAG(*.bag)");
	QString filename = QFileDialog::getOpenFileName(window, QString("Open bag file"), QDir::currentPath(), 
	 	QString("BAG files (*.bag);; All files (*.*)"), &selfilter);
	
	if (filename.isEmpty()) {
	  QMessageBox::critical(ui.centralwidget, "Error", "Could not open file");
          return;
	} else {
	  char command[256];
	  sprintf(command, "gnome-terminal -e 'bash -c \"rosbag play %s; exec bash\" '", filename.toUtf8().constData());
	  system(command);
	}
}

static QImage CvMatToQImage(const cv::Mat& mat) {
  // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type() == CV_8UC1) {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; i++)
            colorTable.push_back(qRgb(i, i, i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(mat.type() == CV_8UC3) {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img;
    } else {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

static void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  ui.labelFront->setPixmap(QPixmap::fromImage(CvMatToQImage(cv_ptr->image)));
}

static void backCameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  ui.labelbottom->setPixmap(QPixmap::fromImage(CvMatToQImage(cv_ptr->image)));
}

//Callback when data input source is changed from selection
static void source_selected(int index) {
  switch(index) {
  case 0: //Simulation
    CHANGE_TOPIC(sub1, "/bumblebee/camera1", frontCameraCallback);
    CHANGE_TOPIC(sub2, "/bumblebee/camera2", backCameraCallback);
    break;
  case 1: //Bag file
    break;
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "auv_gui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);
	
	//Events Handler
	QObject::connect(ui.bottomfilter, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), selectBottomFilter);
	QObject::connect(ui.actionOpen, &QAction::triggered, openFile);
	QObject::connect(ui.source_ddm, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), source_selected);

	window->show();

	ros::NodeHandle node;
	image_transport::ImageTransport it1(node);
	it = &it1;

	sub1 = it1.subscribe("/bumblebee/camera1", 1, frontCameraCallback);
	sub2 = it1.subscribe("/bumblebee/camera2", 1, backCameraCallback);
	
	ros::AsyncSpinner spinner(4);
	spinner.start();

	return app.exec();
}

