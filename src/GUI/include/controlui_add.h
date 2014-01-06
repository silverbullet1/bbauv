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

#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <vector>

using namespace std;

static Ui::ControlSysUI ui;
static QMainWindow *window;

static void initialize();
static void initialiseParameters();
static string getdate();
static void saveFile();
static void openFile();
static void fire();
static void enableButton();
static void disableButton();