#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLineEdit>
#include <QVector>

#include "../src/controlui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <msgs/ControllerAction.h>

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

void initialiseDefault();
void initialiseParameters();
void subscribeToData();
string getdate();
void saveFile();
void openFile();
void fire();
void enableButton();
void disableButton();
void graph_test();



//Functions for the subscribers
void setpt_val_callback(const std_msgs::Float32::ConstPtr& msg);
void sensor_val_callback(const std_msgs::Float32::ConstPtr& msg);
void error_val_callback(const std_msgs::Float32::ConstPtr& msg);
void KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
void KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
void KD_val_callback(const std_msgs::Float32::ConstPtr& msg);
void output_val_callback(const std_msgs::Float32::ConstPtr& msg);
void thruster_val_callback(const std_msgs::Float32::ConstPtr& msg);
void dof_val_callback(const std_msgs::String::ConstPtr& msg);
void goal_val_callback(const std_msgs::Float32::ConstPtr& msg);

void fwdcheck_callback(const std_msgs::Bool::ConstPtr& msg);
void fwd_val_callback(const std_msgs::Float32::ConstPtr& msg);
void depthcheck_callback(const std_msgs::Bool::ConstPtr& msg);
void depth_val_callback(const std_msgs::Float32::ConstPtr& msg);
void yawcheck_callback(const std_msgs::Bool::ConstPtr& msg);
void yaw_val_callback(const std_msgs::Float32::ConstPtr& msg);
void smcheck_callback(const std_msgs::Bool::ConstPtr& msg);
void sm_val_callback(const std_msgs::Float32::ConstPtr& msg);

void actmin_val_callback(const std_msgs::Float32::ConstPtr& msg);
void actmax_val_callback(const std_msgs::Float32::ConstPtr& msg);
void con_KP_val_callback(const std_msgs::Float32::ConstPtr& msg);
void con_KI_val_callback(const std_msgs::Float32::ConstPtr& msg);
void con_KD_val_callback(const std_msgs::Float32::ConstPtr& msg);

void setpt_x_callback(const std_msgs::Float32::ConstPtr& msg);
void setpt_y_callback(const std_msgs::Float32::ConstPtr& msg);
void sensor_x_callback(const std_msgs::Float32::ConstPtr& msg);
void sensor_y_callback(const std_msgs::Float32::ConstPtr& msg);
