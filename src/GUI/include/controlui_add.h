#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLineEdit>

#include "../src/controlui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <sstream>

using namespace std;

static Ui::ControlSysUI ui;
static QMainWindow *window;

static void initialiseDefault();
static void initialiseParameters();
static void subscribeToData();
static string getdate();
static void saveFile();
static void openFile();
static void fire();
static void enableButton();
static void disableButton();

//Functions for the subscribers
static void setpt_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void sensor_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void error_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void KD_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void output_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void thruster_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void dof_val_callback(const std_msgs::String::ConstPtr& msg);
static void goal_val_callback(const std_msgs::Float32::ConstPtr& msg);

static void fwdcheck_callback(const std_msgs::Bool::ConstPtr& msg);
static void fwd_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void depthcheck_callback(const std_msgs::Bool::ConstPtr& msg);
static void depth_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void yawcheck_callback(const std_msgs::Bool::ConstPtr& msg);
static void yaw_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void smcheck_callback(const std_msgs::Bool::ConstPtr& msg);
static void sm_val_callback(const std_msgs::Float32::ConstPtr& msg);

static void actmin_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void actmax_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void con_KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void con_KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
static void con_KD_val_callback(const std_msgs::Float32::ConstPtr& msg);