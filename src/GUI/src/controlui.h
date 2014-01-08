/********************************************************************************
** Form generated from reading UI file 'ControlUI.ui'
**
** Created by: Qt User Interface Compiler version 5.1.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef CONTROLUI_H
#define CONTROLUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_ControlSysUI
{
public:
    QAction *actionSave;
    QAction *actionOpen;
    QAction *actionQuit;
    QWidget *centralwidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *graph;
    QCustomPlot *graph_canvas;
    QLabel *graph_label;
    QFrame *telemetry_box;
    QLabel *setpt_label;
    QLabel *sensor_label;
    QLabel *error_label;
    QLabel *KP_label;
    QLabel *KI_label;
    QLabel *KD_label;
    QLabel *output_label;
    QLabel *thruster_label;
    QLabel *setpt_val;
    QLabel *sensor_val;
    QLabel *error_val;
    QLabel *KP_val;
    QLabel *KI_val;
    QLabel *KD_val;
    QLabel *output_val;
    QLabel *thruster_val;
    QLabel *telemetry_label;
    QLabel *DOF_label;
    QLabel *Goal_label;
    QLineEdit *goal_val;
    QFrame *controparam_frame;
    QLabel *actmin_label;
    QLabel *actmax_label;
    QLabel *con_KD_label;
    QLabel *con_KI_label;
    QLabel *Controlpara_label;
    QLabel *con_KP_label;
    QLabel *con_KP_val;
    QLabel *con_KI_val;
    QLabel *con_KD_val;
    QLabel *actmin_val;
    QLabel *actmax_val;
    QFrame *advanced_frame;
    QLabel *Fwd_label;
    QCheckBox *fwd_check;
    QLineEdit *fwd_val;
    QLabel *Depth_label;
    QCheckBox *depth_check;
    QLineEdit *depth_val;
    QLineEdit *yaw_val;
    QLabel *yaw_label;
    QCheckBox *yaw_check;
    QLineEdit *sm_val;
    QCheckBox *sm_check;
    QLabel *sm_label;
    QPushButton *enabledButton;
    QPushButton *disabledButton;
    QLabel *Advanced_label;
    QComboBox *dof_comboBox;
    QLabel *graphvalues;
    QFrame *frame;
    QPushButton *fireButton;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ControlSysUI)
    {
        if (ControlSysUI->objectName().isEmpty())
            ControlSysUI->setObjectName(QStringLiteral("ControlSysUI"));
        ControlSysUI->resize(854, 571);
        ControlSysUI->setTabShape(QTabWidget::Rounded);
        actionSave = new QAction(ControlSysUI);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionOpen = new QAction(ControlSysUI);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionQuit = new QAction(ControlSysUI);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        centralwidget = new QWidget(ControlSysUI);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(20, 40, 341, 411));
        graph = new QHBoxLayout(horizontalLayoutWidget);
        graph->setObjectName(QStringLiteral("graph"));
        graph->setContentsMargins(0, 0, 0, 0);
        graph_canvas = new QCustomPlot(horizontalLayoutWidget);
        graph_canvas->setObjectName(QStringLiteral("graph_canvas"));
        graph_canvas->setCursor(QCursor(Qt::CrossCursor));

        graph->addWidget(graph_canvas);

        graph_label = new QLabel(centralwidget);
        graph_label->setObjectName(QStringLiteral("graph_label"));
        graph_label->setGeometry(QRect(20, 10, 66, 17));
        telemetry_box = new QFrame(centralwidget);
        telemetry_box->setObjectName(QStringLiteral("telemetry_box"));
        telemetry_box->setGeometry(QRect(380, 10, 441, 131));
        telemetry_box->setFrameShape(QFrame::StyledPanel);
        setpt_label = new QLabel(telemetry_box);
        setpt_label->setObjectName(QStringLiteral("setpt_label"));
        setpt_label->setGeometry(QRect(20, 40, 51, 17));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        setpt_label->setFont(font);
        sensor_label = new QLabel(telemetry_box);
        sensor_label->setObjectName(QStringLiteral("sensor_label"));
        sensor_label->setGeometry(QRect(20, 70, 51, 17));
        sensor_label->setFont(font);
        error_label = new QLabel(telemetry_box);
        error_label->setObjectName(QStringLiteral("error_label"));
        error_label->setGeometry(QRect(20, 100, 51, 17));
        error_label->setFont(font);
        KP_label = new QLabel(telemetry_box);
        KP_label->setObjectName(QStringLiteral("KP_label"));
        KP_label->setGeometry(QRect(170, 40, 51, 17));
        KP_label->setFont(font);
        KI_label = new QLabel(telemetry_box);
        KI_label->setObjectName(QStringLiteral("KI_label"));
        KI_label->setGeometry(QRect(170, 70, 51, 17));
        KI_label->setFont(font);
        KD_label = new QLabel(telemetry_box);
        KD_label->setObjectName(QStringLiteral("KD_label"));
        KD_label->setGeometry(QRect(170, 100, 51, 17));
        KD_label->setFont(font);
        output_label = new QLabel(telemetry_box);
        output_label->setObjectName(QStringLiteral("output_label"));
        output_label->setGeometry(QRect(290, 40, 51, 17));
        output_label->setFont(font);
        thruster_label = new QLabel(telemetry_box);
        thruster_label->setObjectName(QStringLiteral("thruster_label"));
        thruster_label->setGeometry(QRect(290, 80, 61, 17));
        thruster_label->setFont(font);
        setpt_val = new QLabel(telemetry_box);
        setpt_val->setObjectName(QStringLiteral("setpt_val"));
        setpt_val->setGeometry(QRect(80, 40, 66, 17));
        sensor_val = new QLabel(telemetry_box);
        sensor_val->setObjectName(QStringLiteral("sensor_val"));
        sensor_val->setGeometry(QRect(80, 70, 66, 17));
        error_val = new QLabel(telemetry_box);
        error_val->setObjectName(QStringLiteral("error_val"));
        error_val->setGeometry(QRect(80, 100, 66, 17));
        KP_val = new QLabel(telemetry_box);
        KP_val->setObjectName(QStringLiteral("KP_val"));
        KP_val->setGeometry(QRect(210, 40, 66, 17));
        KI_val = new QLabel(telemetry_box);
        KI_val->setObjectName(QStringLiteral("KI_val"));
        KI_val->setGeometry(QRect(210, 70, 66, 17));
        KD_val = new QLabel(telemetry_box);
        KD_val->setObjectName(QStringLiteral("KD_val"));
        KD_val->setGeometry(QRect(210, 100, 66, 17));
        output_val = new QLabel(telemetry_box);
        output_val->setObjectName(QStringLiteral("output_val"));
        output_val->setGeometry(QRect(360, 40, 66, 17));
        thruster_val = new QLabel(telemetry_box);
        thruster_val->setObjectName(QStringLiteral("thruster_val"));
        thruster_val->setGeometry(QRect(360, 80, 66, 17));
        telemetry_label = new QLabel(telemetry_box);
        telemetry_label->setObjectName(QStringLiteral("telemetry_label"));
        telemetry_label->setGeometry(QRect(20, 10, 81, 17));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setItalic(true);
        font1.setUnderline(true);
        font1.setWeight(75);
        telemetry_label->setFont(font1);
        DOF_label = new QLabel(centralwidget);
        DOF_label->setObjectName(QStringLiteral("DOF_label"));
        DOF_label->setGeometry(QRect(390, 160, 51, 17));
        Goal_label = new QLabel(centralwidget);
        Goal_label->setObjectName(QStringLiteral("Goal_label"));
        Goal_label->setGeometry(QRect(390, 200, 51, 17));
        goal_val = new QLineEdit(centralwidget);
        goal_val->setObjectName(QStringLiteral("goal_val"));
        goal_val->setGeometry(QRect(440, 200, 101, 27));
        goal_val->setCursor(QCursor(Qt::IBeamCursor));
        controparam_frame = new QFrame(centralwidget);
        controparam_frame->setObjectName(QStringLiteral("controparam_frame"));
        controparam_frame->setGeometry(QRect(380, 370, 441, 131));
        controparam_frame->setFrameShape(QFrame::StyledPanel);
        controparam_frame->setFrameShadow(QFrame::Raised);
        actmin_label = new QLabel(controparam_frame);
        actmin_label->setObjectName(QStringLiteral("actmin_label"));
        actmin_label->setGeometry(QRect(170, 40, 61, 17));
        actmin_label->setFont(font);
        actmax_label = new QLabel(controparam_frame);
        actmax_label->setObjectName(QStringLiteral("actmax_label"));
        actmax_label->setGeometry(QRect(170, 80, 61, 17));
        actmax_label->setFont(font);
        con_KD_label = new QLabel(controparam_frame);
        con_KD_label->setObjectName(QStringLiteral("con_KD_label"));
        con_KD_label->setGeometry(QRect(20, 100, 51, 17));
        con_KD_label->setFont(font);
        con_KI_label = new QLabel(controparam_frame);
        con_KI_label->setObjectName(QStringLiteral("con_KI_label"));
        con_KI_label->setGeometry(QRect(20, 70, 51, 17));
        con_KI_label->setFont(font);
        Controlpara_label = new QLabel(controparam_frame);
        Controlpara_label->setObjectName(QStringLiteral("Controlpara_label"));
        Controlpara_label->setGeometry(QRect(20, 10, 151, 17));
        con_KP_label = new QLabel(controparam_frame);
        con_KP_label->setObjectName(QStringLiteral("con_KP_label"));
        con_KP_label->setGeometry(QRect(20, 40, 51, 17));
        con_KP_label->setFont(font);
        con_KP_val = new QLabel(controparam_frame);
        con_KP_val->setObjectName(QStringLiteral("con_KP_val"));
        con_KP_val->setGeometry(QRect(60, 40, 66, 17));
        con_KI_val = new QLabel(controparam_frame);
        con_KI_val->setObjectName(QStringLiteral("con_KI_val"));
        con_KI_val->setGeometry(QRect(60, 70, 66, 17));
        con_KD_val = new QLabel(controparam_frame);
        con_KD_val->setObjectName(QStringLiteral("con_KD_val"));
        con_KD_val->setGeometry(QRect(60, 100, 66, 17));
        actmin_val = new QLabel(controparam_frame);
        actmin_val->setObjectName(QStringLiteral("actmin_val"));
        actmin_val->setGeometry(QRect(250, 40, 66, 17));
        actmax_val = new QLabel(controparam_frame);
        actmax_val->setObjectName(QStringLiteral("actmax_val"));
        actmax_val->setGeometry(QRect(250, 80, 66, 17));
        advanced_frame = new QFrame(centralwidget);
        advanced_frame->setObjectName(QStringLiteral("advanced_frame"));
        advanced_frame->setGeometry(QRect(560, 150, 261, 211));
        advanced_frame->setFrameShape(QFrame::StyledPanel);
        advanced_frame->setFrameShadow(QFrame::Raised);
        Fwd_label = new QLabel(advanced_frame);
        Fwd_label->setObjectName(QStringLiteral("Fwd_label"));
        Fwd_label->setGeometry(QRect(20, 40, 51, 17));
        fwd_check = new QCheckBox(advanced_frame);
        fwd_check->setObjectName(QStringLiteral("fwd_check"));
        fwd_check->setGeometry(QRect(80, 40, 97, 22));
        fwd_val = new QLineEdit(advanced_frame);
        fwd_val->setObjectName(QStringLiteral("fwd_val"));
        fwd_val->setGeometry(QRect(130, 40, 101, 27));
        Depth_label = new QLabel(advanced_frame);
        Depth_label->setObjectName(QStringLiteral("Depth_label"));
        Depth_label->setGeometry(QRect(20, 70, 51, 17));
        depth_check = new QCheckBox(advanced_frame);
        depth_check->setObjectName(QStringLiteral("depth_check"));
        depth_check->setGeometry(QRect(80, 70, 97, 22));
        depth_val = new QLineEdit(advanced_frame);
        depth_val->setObjectName(QStringLiteral("depth_val"));
        depth_val->setGeometry(QRect(130, 70, 101, 27));
        yaw_val = new QLineEdit(advanced_frame);
        yaw_val->setObjectName(QStringLiteral("yaw_val"));
        yaw_val->setGeometry(QRect(130, 100, 101, 27));
        yaw_label = new QLabel(advanced_frame);
        yaw_label->setObjectName(QStringLiteral("yaw_label"));
        yaw_label->setGeometry(QRect(20, 100, 51, 17));
        yaw_check = new QCheckBox(advanced_frame);
        yaw_check->setObjectName(QStringLiteral("yaw_check"));
        yaw_check->setGeometry(QRect(80, 100, 97, 22));
        sm_val = new QLineEdit(advanced_frame);
        sm_val->setObjectName(QStringLiteral("sm_val"));
        sm_val->setGeometry(QRect(130, 130, 101, 27));
        sm_check = new QCheckBox(advanced_frame);
        sm_check->setObjectName(QStringLiteral("sm_check"));
        sm_check->setGeometry(QRect(80, 130, 97, 22));
        sm_label = new QLabel(advanced_frame);
        sm_label->setObjectName(QStringLiteral("sm_label"));
        sm_label->setGeometry(QRect(20, 130, 51, 17));
        enabledButton = new QPushButton(advanced_frame);
        enabledButton->setObjectName(QStringLiteral("enabledButton"));
        enabledButton->setGeometry(QRect(20, 170, 98, 31));
        disabledButton = new QPushButton(advanced_frame);
        disabledButton->setObjectName(QStringLiteral("disabledButton"));
        disabledButton->setGeometry(QRect(140, 170, 98, 31));
        Advanced_label = new QLabel(centralwidget);
        Advanced_label->setObjectName(QStringLiteral("Advanced_label"));
        Advanced_label->setGeometry(QRect(580, 160, 151, 17));
        Advanced_label->setTextFormat(Qt::RichText);
        dof_comboBox = new QComboBox(centralwidget);
        dof_comboBox->setObjectName(QStringLiteral("dof_comboBox"));
        dof_comboBox->setGeometry(QRect(440, 160, 91, 27));
        graphvalues = new QLabel(centralwidget);
        graphvalues->setObjectName(QStringLiteral("graphvalues"));
        graphvalues->setGeometry(QRect(20, 470, 321, 21));
        QFont font2;
        font2.setPointSize(14);
        font2.setBold(true);
        font2.setWeight(75);
        graphvalues->setFont(font2);
        graphvalues->setTextFormat(Qt::AutoText);
        frame = new QFrame(centralwidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(380, 150, 171, 211));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        fireButton = new QPushButton(frame);
        fireButton->setObjectName(QStringLiteral("fireButton"));
        fireButton->setGeometry(QRect(20, 100, 131, 27));
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(170, 85, 127, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(255, 127, 191, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(212, 106, 159, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(85, 42, 63, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(113, 56, 84, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        QBrush brush6(QColor(255, 255, 255, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        QBrush brush7(QColor(212, 170, 191, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush7);
        QBrush brush8(QColor(255, 255, 220, 255));
        brush8.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        fireButton->setPalette(palette);
        fireButton->setAutoFillBackground(true);
        fireButton->setCheckable(false);
        ControlSysUI->setCentralWidget(centralwidget);
        frame->raise();
        horizontalLayoutWidget->raise();
        graph_label->raise();
        telemetry_box->raise();
        DOF_label->raise();
        Goal_label->raise();
        goal_val->raise();
        controparam_frame->raise();
        advanced_frame->raise();
        Advanced_label->raise();
        dof_comboBox->raise();
        graphvalues->raise();
        menubar = new QMenuBar(ControlSysUI);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 854, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        ControlSysUI->setMenuBar(menubar);
        statusbar = new QStatusBar(ControlSysUI);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        ControlSysUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionSave);
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionQuit);

        retranslateUi(ControlSysUI);
        QObject::connect(actionQuit, SIGNAL(triggered()), ControlSysUI, SLOT(close()));

        QMetaObject::connectSlotsByName(ControlSysUI);
    } // setupUi

    void retranslateUi(QMainWindow *ControlSysUI)
    {
        ControlSysUI->setWindowTitle(QApplication::translate("ControlSysUI", "Control System UI", 0));
        actionSave->setText(QApplication::translate("ControlSysUI", "&Save", 0));
        actionSave->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+S", 0));
        actionOpen->setText(QApplication::translate("ControlSysUI", "&Open", 0));
        actionOpen->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+O", 0));
        actionQuit->setText(QApplication::translate("ControlSysUI", "&Quit", 0));
        actionQuit->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+Q", 0));
        graph_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Graph</span></p></body></html>", 0));
        setpt_label->setText(QApplication::translate("ControlSysUI", "Set pt", 0));
        sensor_label->setText(QApplication::translate("ControlSysUI", "Sensor", 0));
        error_label->setText(QApplication::translate("ControlSysUI", "Error", 0));
        KP_label->setText(QApplication::translate("ControlSysUI", "KP", 0));
        KI_label->setText(QApplication::translate("ControlSysUI", "KI", 0));
        KD_label->setText(QApplication::translate("ControlSysUI", "KD", 0));
        output_label->setText(QApplication::translate("ControlSysUI", "Output", 0));
        thruster_label->setText(QApplication::translate("ControlSysUI", "Thruster", 0));
        setpt_val->setText(QApplication::translate("ControlSysUI", "setpt", 0));
        sensor_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        error_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        KP_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        KI_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        KD_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        output_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        thruster_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        telemetry_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-style:normal;\">Telemetry</span></p></body></html>", 0));
        DOF_label->setText(QApplication::translate("ControlSysUI", "DOF", 0));
        Goal_label->setText(QApplication::translate("ControlSysUI", "Goal", 0));
        actmin_label->setText(QApplication::translate("ControlSysUI", "Act_Min", 0));
        actmax_label->setText(QApplication::translate("ControlSysUI", "Act_Max", 0));
        con_KD_label->setText(QApplication::translate("ControlSysUI", "KD", 0));
        con_KI_label->setText(QApplication::translate("ControlSysUI", "KI", 0));
        Controlpara_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; text-decoration: underline;\">Control Parameters</span></p></body></html>", 0));
        con_KP_label->setText(QApplication::translate("ControlSysUI", "KP", 0));
        con_KP_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        con_KI_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        con_KD_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        actmin_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        actmax_val->setText(QApplication::translate("ControlSysUI", "TextLabel", 0));
        Fwd_label->setText(QApplication::translate("ControlSysUI", "Fwd", 0));
        fwd_check->setText(QString());
        Depth_label->setText(QApplication::translate("ControlSysUI", "Depth", 0));
        depth_check->setText(QString());
        yaw_label->setText(QApplication::translate("ControlSysUI", "Yaw", 0));
        yaw_check->setText(QString());
        sm_check->setText(QString());
        sm_label->setText(QApplication::translate("ControlSysUI", "SM", 0));
        enabledButton->setText(QApplication::translate("ControlSysUI", "&Enabled", 0));
        enabledButton->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+E", 0));
        disabledButton->setText(QApplication::translate("ControlSysUI", "&Disabled", 0));
        disabledButton->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+D", 0));
        Advanced_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; text-decoration: underline;\">Advanced</span></p></body></html>", 0));
        dof_comboBox->clear();
        dof_comboBox->insertItems(0, QStringList()
         << QApplication::translate("ControlSysUI", "pos_X", 0)
         << QApplication::translate("ControlSysUI", "pos_Y", 0)
         << QApplication::translate("ControlSysUI", "pos_Z", 0)
        );
        graphvalues->setText(QApplication::translate("ControlSysUI", "Graph values:       x:       y:", 0));
        fireButton->setText(QApplication::translate("ControlSysUI", "Fi&re", 0));
        fireButton->setShortcut(QApplication::translate("ControlSysUI", "Ctrl+R", 0));
        menuFile->setTitle(QApplication::translate("ControlSysUI", "&File", 0));
    } // retranslateUi

};

namespace Ui {
    class ControlSysUI: public Ui_ControlSysUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // CONTROLUI_H
