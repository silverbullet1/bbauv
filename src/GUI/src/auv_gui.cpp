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

static Ui::Vision ui;
static QMainWindow *window;

static void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

static void selectBottomFilter(int selectedIndex) {
	ui.bottomfilter->setItemText(selectedIndex,"Hello");
}

static void openFile(bool open){
	//Apparently QMainWIndow needs to be casted to a QWidget? 
	//QFileDialog to open file
	//exit(2);//didnt even come in this method..
        //QWidget *widget = new QWidget;
	QString selfilter = QString("BAG(*.bag)");
	QString filename = QFileDialog::getOpenFileName(window, QString("Open bag file"), QDir::currentPath(), 
	 	QString("BAG files (*.bag);; All files (*.*)"), &selfilter);
	
	// if (!filename.isNull()) { qDebug.toAscii(); }
	// //Try to run the bag file from a new terminal in rosrun
	// char command[500];
	// strcpy(command, "gnome-terminal -e 'bash -c " + "\"" + "rosbag play "  + filename + "; exec bash\"" + "'");
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "auv_gui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);
	
	QObject::connect(ui.bottomfilter, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), selectBottomFilter);
	QObject::connect(ui.actionOpen, &QAction::triggered, openFile);

	window->show();

	ros::NodeHandle node;

	image_transport::ImageTransport it1(node);
	image_transport::Subscriber sub1 = it1.subscribe("/bumblebee/camera1", 1, frontCameraCallback);
	
	ros::AsyncSpinner spinner(4);
	spinner.start();

	return app.exec();
}

