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
#include <string.h>
#include <stdlib.h>

#include "filters.h"

using namespace std;

//Macro to change subscriber's topic during run time
#define CHANGE_TOPIC(sub, topic, callback)	\
		(sub = it->subscribe(topic, 1, callback))

enum camera_t { FRONT, BOTTOM };

static Ui::Vision ui;
static QMainWindow *window;

static image_transport::Subscriber sub1, sub2;
static image_transport::ImageTransport *it;

static void update_filter(camera_t camera, cv::Mat image);


static void openFile(bool open){
	QString selfilter = QString("BAG(*.bag)");
	QString filename = QFileDialog::getOpenFileName(window, QString("Open bag file"), QDir::currentPath(), 
	QString("BAG files (*.bag);; All files (*.*)"), &selfilter);

	string filename_string = filename.toUtf8().constData();

	if (!filename_string.empty()){
		char command[256];
		sprintf(command, "gnome-terminal -e 'bash -c \"cd launch; roslaunch uncompressbags.launch bagfile:=%s; exec bash\" '", filename_string.c_str());
		//sprintf(command, "gnome-terminal -e 'bash -c \"rosbag play %s; exec bash\" '", filename_string.c_str());
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
	cv::Mat smallerImage(ui.labelFront->size().height(), ui.labelFront->size().width(), CV_8UC3, cv::Scalar(0,0,0));
	//std::cout << ui.labelFront->size().width() << " " << ui.labelFront->size().height() << std::endl;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		cv::resize(cv_ptr->image, smallerImage, smallerImage.size());
		update_filter(FRONT, smallerImage);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	ui.labelFront->setPixmap(QPixmap::fromImage(CvMatToQImage(smallerImage)));
}

static void bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat smallerImage(ui.labelFront->size().height(), ui.labelFront->size().width(), CV_8UC3, cv::Scalar(0,0,0));
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		cv::resize(cv_ptr->image, smallerImage, smallerImage.size());
		update_filter(BOTTOM, smallerImage);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	ui.labelBottom->setPixmap(QPixmap::fromImage(CvMatToQImage(cv_ptr->image)));
}

//Callback when data input source is changed from selection
//Just need to remap the topic
static void source_selected(int index) {
	switch(index) {
	case 1: //Bag file must run uncompress bags
		CHANGE_TOPIC(sub1, "/front_right", frontCameraCallback);
		CHANGE_TOPIC(sub2, "/bottomcam", bottomCameraCallback);
		break;
	}
	CHANGE_TOPIC(sub1, "/bumblebee/camera1", frontCameraCallback);
	CHANGE_TOPIC(sub2, "/bumblebee/camera2", bottomCameraCallback);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "auv_gui");

	//Initiate QAppication and UI
	QApplication app(argc, argv);
	window = new QMainWindow;
	ui.setupUi(window);
	window->setFixedSize(window->geometry().width(), window->geometry().height());

	//Events Handlers
	//QObject::connect(ui.bottomfilter, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), selectBottomFilter);
	QObject::connect(ui.actionOpen, &QAction::triggered, openFile);
	QObject::connect(ui.source_ddm, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), source_selected);

	window->show();

	ros::NodeHandle node;
	image_transport::ImageTransport it1(node);
	it = &it1;

	sub1 = it1.subscribe("/bumblebee/camera1", 1, frontCameraCallback);
	sub2 = it1.subscribe("/bumblebee/camera2", 1, bottomCameraCallback);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	return app.exec();
}

static void update_filter(camera_t camera, cv::Mat image) {
	switch (camera) {
		case FRONT: {
			cv::Mat front_out = (front_filters[ui.frontfilter->currentIndex()])(image);
			ui.labelFrontFiltered->setPixmap(QPixmap::fromImage(CvMatToQImage(front_out)));
		}
			break;
		case BOTTOM: {
			cv::Mat bottom_out = (bottom_filters[ui.bottomfilter->currentIndex()])(image);
			ui.labelBottomFiltered->setPixmap(QPixmap::fromImage(CvMatToQImage(bottom_out)));
		}
			break;
	}
}