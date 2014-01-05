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
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

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
    QLabel *graph_label;
    QLabel *telemetry_label;
    QFrame *telemetry_box;
    QLineEdit *setpt_val;
    QLabel *setpt_label;
    QLineEdit *sensor_val;
    QLabel *sensor_label;
    QLineEdit *error_val;
    QLabel *error_label;
    QLabel *KP_label;
    QLineEdit *KP_val;
    QLabel *KI_label;
    QLineEdit *KP_val_2;
    QLabel *KD_label;
    QLineEdit *KD_val;
    QLabel *output_label;
    QLineEdit *output_val;
    QLabel *thruster_label;
    QLineEdit *thruster_val;
    QPushButton *saveButton;
    QLabel *DOF_label;
    QSpinBox *DOF_spinbox;
    QLabel *Goal_label;
    QLineEdit *goal_val;
    QFrame *controparam_frame;
    QLineEdit *actmax_val;
    QLabel *actmin_label;
    QLabel *actmax_label;
    QLabel *con_KD_label;
    QLabel *con_KI_label;
    QLineEdit *actmin_val;
    QLabel *Controlpara_label;
    QLineEdit *con_KP_val_2;
    QLabel *con_KP_label;
    QLineEdit *con_KD_val;
    QLineEdit *con_KP_val;
    QFrame *advanced_frame;
    QLabel *Fwd_label;
    QCheckBox *fwd_check;
    QLineEdit *fwd_val;
    QLabel *Depth_label;
    QCheckBox *Depth_check;
    QLineEdit *Depth_val;
    QLineEdit *yaw_val;
    QLabel *yaw_label;
    QCheckBox *yaw_check;
    QLineEdit *sm_val;
    QCheckBox *sm_check;
    QLabel *sm_label;
    QPushButton *enabledButton;
    QPushButton *disabledButton;
    QLabel *Advanced_label;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ControlSysUI)
    {
        if (ControlSysUI->objectName().isEmpty())
            ControlSysUI->setObjectName(QStringLiteral("ControlSysUI"));
        ControlSysUI->resize(846, 570);
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
        graph_label = new QLabel(centralwidget);
        graph_label->setObjectName(QStringLiteral("graph_label"));
        graph_label->setGeometry(QRect(20, 10, 66, 17));
        telemetry_label = new QLabel(centralwidget);
        telemetry_label->setObjectName(QStringLiteral("telemetry_label"));
        telemetry_label->setGeometry(QRect(380, 10, 81, 17));
        telemetry_box = new QFrame(centralwidget);
        telemetry_box->setObjectName(QStringLiteral("telemetry_box"));
        telemetry_box->setGeometry(QRect(380, 30, 431, 111));
        setpt_val = new QLineEdit(telemetry_box);
        setpt_val->setObjectName(QStringLiteral("setpt_val"));
        setpt_val->setGeometry(QRect(70, 10, 61, 27));
        setpt_label = new QLabel(telemetry_box);
        setpt_label->setObjectName(QStringLiteral("setpt_label"));
        setpt_label->setGeometry(QRect(10, 10, 51, 17));
        sensor_val = new QLineEdit(telemetry_box);
        sensor_val->setObjectName(QStringLiteral("sensor_val"));
        sensor_val->setGeometry(QRect(70, 40, 61, 27));
        sensor_label = new QLabel(telemetry_box);
        sensor_label->setObjectName(QStringLiteral("sensor_label"));
        sensor_label->setGeometry(QRect(10, 40, 51, 17));
        error_val = new QLineEdit(telemetry_box);
        error_val->setObjectName(QStringLiteral("error_val"));
        error_val->setGeometry(QRect(70, 70, 61, 27));
        error_label = new QLabel(telemetry_box);
        error_label->setObjectName(QStringLiteral("error_label"));
        error_label->setGeometry(QRect(10, 70, 51, 17));
        KP_label = new QLabel(telemetry_box);
        KP_label->setObjectName(QStringLiteral("KP_label"));
        KP_label->setGeometry(QRect(150, 10, 51, 17));
        KP_val = new QLineEdit(telemetry_box);
        KP_val->setObjectName(QStringLiteral("KP_val"));
        KP_val->setGeometry(QRect(190, 10, 61, 27));
        KI_label = new QLabel(telemetry_box);
        KI_label->setObjectName(QStringLiteral("KI_label"));
        KI_label->setGeometry(QRect(150, 40, 51, 17));
        KP_val_2 = new QLineEdit(telemetry_box);
        KP_val_2->setObjectName(QStringLiteral("KP_val_2"));
        KP_val_2->setGeometry(QRect(190, 40, 61, 27));
        KD_label = new QLabel(telemetry_box);
        KD_label->setObjectName(QStringLiteral("KD_label"));
        KD_label->setGeometry(QRect(150, 70, 51, 17));
        KD_val = new QLineEdit(telemetry_box);
        KD_val->setObjectName(QStringLiteral("KD_val"));
        KD_val->setGeometry(QRect(190, 70, 61, 27));
        output_label = new QLabel(telemetry_box);
        output_label->setObjectName(QStringLiteral("output_label"));
        output_label->setGeometry(QRect(270, 10, 51, 17));
        output_val = new QLineEdit(telemetry_box);
        output_val->setObjectName(QStringLiteral("output_val"));
        output_val->setGeometry(QRect(330, 10, 61, 27));
        thruster_label = new QLabel(telemetry_box);
        thruster_label->setObjectName(QStringLiteral("thruster_label"));
        thruster_label->setGeometry(QRect(270, 50, 61, 17));
        thruster_val = new QLineEdit(telemetry_box);
        thruster_val->setObjectName(QStringLiteral("thruster_val"));
        thruster_val->setGeometry(QRect(330, 50, 61, 27));
        saveButton = new QPushButton(centralwidget);
        saveButton->setObjectName(QStringLiteral("saveButton"));
        saveButton->setGeometry(QRect(710, 490, 98, 27));
        DOF_label = new QLabel(centralwidget);
        DOF_label->setObjectName(QStringLiteral("DOF_label"));
        DOF_label->setGeometry(QRect(390, 150, 51, 17));
        DOF_spinbox = new QSpinBox(centralwidget);
        DOF_spinbox->setObjectName(QStringLiteral("DOF_spinbox"));
        DOF_spinbox->setGeometry(QRect(440, 150, 101, 27));
        Goal_label = new QLabel(centralwidget);
        Goal_label->setObjectName(QStringLiteral("Goal_label"));
        Goal_label->setGeometry(QRect(390, 190, 51, 17));
        goal_val = new QLineEdit(centralwidget);
        goal_val->setObjectName(QStringLiteral("goal_val"));
        goal_val->setGeometry(QRect(440, 190, 101, 27));
        controparam_frame = new QFrame(centralwidget);
        controparam_frame->setObjectName(QStringLiteral("controparam_frame"));
        controparam_frame->setGeometry(QRect(380, 360, 431, 121));
        controparam_frame->setFrameShape(QFrame::StyledPanel);
        controparam_frame->setFrameShadow(QFrame::Raised);
        actmax_val = new QLineEdit(controparam_frame);
        actmax_val->setObjectName(QStringLiteral("actmax_val"));
        actmax_val->setGeometry(QRect(240, 70, 81, 27));
        actmin_label = new QLabel(controparam_frame);
        actmin_label->setObjectName(QStringLiteral("actmin_label"));
        actmin_label->setGeometry(QRect(170, 30, 61, 17));
        actmax_label = new QLabel(controparam_frame);
        actmax_label->setObjectName(QStringLiteral("actmax_label"));
        actmax_label->setGeometry(QRect(170, 70, 61, 17));
        con_KD_label = new QLabel(controparam_frame);
        con_KD_label->setObjectName(QStringLiteral("con_KD_label"));
        con_KD_label->setGeometry(QRect(20, 90, 51, 17));
        con_KI_label = new QLabel(controparam_frame);
        con_KI_label->setObjectName(QStringLiteral("con_KI_label"));
        con_KI_label->setGeometry(QRect(20, 60, 51, 17));
        actmin_val = new QLineEdit(controparam_frame);
        actmin_val->setObjectName(QStringLiteral("actmin_val"));
        actmin_val->setGeometry(QRect(240, 30, 81, 27));
        Controlpara_label = new QLabel(controparam_frame);
        Controlpara_label->setObjectName(QStringLiteral("Controlpara_label"));
        Controlpara_label->setGeometry(QRect(20, 0, 151, 17));
        con_KP_val_2 = new QLineEdit(controparam_frame);
        con_KP_val_2->setObjectName(QStringLiteral("con_KP_val_2"));
        con_KP_val_2->setGeometry(QRect(60, 60, 61, 27));
        con_KP_label = new QLabel(controparam_frame);
        con_KP_label->setObjectName(QStringLiteral("con_KP_label"));
        con_KP_label->setGeometry(QRect(20, 30, 51, 17));
        con_KD_val = new QLineEdit(controparam_frame);
        con_KD_val->setObjectName(QStringLiteral("con_KD_val"));
        con_KD_val->setGeometry(QRect(60, 90, 61, 27));
        con_KP_val = new QLineEdit(controparam_frame);
        con_KP_val->setObjectName(QStringLiteral("con_KP_val"));
        con_KP_val->setGeometry(QRect(60, 30, 61, 27));
        advanced_frame = new QFrame(centralwidget);
        advanced_frame->setObjectName(QStringLiteral("advanced_frame"));
        advanced_frame->setGeometry(QRect(550, 160, 261, 191));
        advanced_frame->setFrameShape(QFrame::StyledPanel);
        advanced_frame->setFrameShadow(QFrame::Raised);
        Fwd_label = new QLabel(advanced_frame);
        Fwd_label->setObjectName(QStringLiteral("Fwd_label"));
        Fwd_label->setGeometry(QRect(10, 10, 51, 17));
        fwd_check = new QCheckBox(advanced_frame);
        fwd_check->setObjectName(QStringLiteral("fwd_check"));
        fwd_check->setGeometry(QRect(50, 10, 97, 22));
        fwd_val = new QLineEdit(advanced_frame);
        fwd_val->setObjectName(QStringLiteral("fwd_val"));
        fwd_val->setGeometry(QRect(100, 10, 101, 27));
        Depth_label = new QLabel(advanced_frame);
        Depth_label->setObjectName(QStringLiteral("Depth_label"));
        Depth_label->setGeometry(QRect(10, 40, 51, 17));
        Depth_check = new QCheckBox(advanced_frame);
        Depth_check->setObjectName(QStringLiteral("Depth_check"));
        Depth_check->setGeometry(QRect(50, 40, 97, 22));
        Depth_val = new QLineEdit(advanced_frame);
        Depth_val->setObjectName(QStringLiteral("Depth_val"));
        Depth_val->setGeometry(QRect(100, 40, 101, 27));
        yaw_val = new QLineEdit(advanced_frame);
        yaw_val->setObjectName(QStringLiteral("yaw_val"));
        yaw_val->setGeometry(QRect(100, 70, 101, 27));
        yaw_label = new QLabel(advanced_frame);
        yaw_label->setObjectName(QStringLiteral("yaw_label"));
        yaw_label->setGeometry(QRect(10, 70, 51, 17));
        yaw_check = new QCheckBox(advanced_frame);
        yaw_check->setObjectName(QStringLiteral("yaw_check"));
        yaw_check->setGeometry(QRect(50, 70, 97, 22));
        sm_val = new QLineEdit(advanced_frame);
        sm_val->setObjectName(QStringLiteral("sm_val"));
        sm_val->setGeometry(QRect(100, 100, 101, 27));
        sm_check = new QCheckBox(advanced_frame);
        sm_check->setObjectName(QStringLiteral("sm_check"));
        sm_check->setGeometry(QRect(50, 100, 97, 22));
        sm_label = new QLabel(advanced_frame);
        sm_label->setObjectName(QStringLiteral("sm_label"));
        sm_label->setGeometry(QRect(10, 100, 51, 17));
        enabledButton = new QPushButton(advanced_frame);
        enabledButton->setObjectName(QStringLiteral("enabledButton"));
        enabledButton->setGeometry(QRect(10, 150, 98, 27));
        disabledButton = new QPushButton(advanced_frame);
        disabledButton->setObjectName(QStringLiteral("disabledButton"));
        disabledButton->setGeometry(QRect(130, 150, 98, 27));
        Advanced_label = new QLabel(centralwidget);
        Advanced_label->setObjectName(QStringLiteral("Advanced_label"));
        Advanced_label->setGeometry(QRect(550, 140, 151, 17));
        Advanced_label->setTextFormat(Qt::RichText);
        ControlSysUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ControlSysUI);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 846, 25));
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

        QMetaObject::connectSlotsByName(ControlSysUI);
    } // setupUi

    void retranslateUi(QMainWindow *ControlSysUI)
    {
        ControlSysUI->setWindowTitle(QApplication::translate("ControlSysUI", "Control System UI", 0));
        actionSave->setText(QApplication::translate("ControlSysUI", "Save", 0));
        actionOpen->setText(QApplication::translate("ControlSysUI", "Open", 0));
        actionQuit->setText(QApplication::translate("ControlSysUI", "Quit", 0));
        graph_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Graph</span></p></body></html>", 0));
        telemetry_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Telemetry</span></p></body></html>", 0));
        setpt_label->setText(QApplication::translate("ControlSysUI", "Set pt", 0));
        sensor_val->setText(QString());
        sensor_label->setText(QApplication::translate("ControlSysUI", "Sensor", 0));
        error_label->setText(QApplication::translate("ControlSysUI", "Error", 0));
        KP_label->setText(QApplication::translate("ControlSysUI", "KP", 0));
        KI_label->setText(QApplication::translate("ControlSysUI", "KI", 0));
        KD_label->setText(QApplication::translate("ControlSysUI", "KD", 0));
        output_label->setText(QApplication::translate("ControlSysUI", "Output", 0));
        thruster_label->setText(QApplication::translate("ControlSysUI", "Thruster", 0));
        saveButton->setText(QApplication::translate("ControlSysUI", "Save", 0));
        DOF_label->setText(QApplication::translate("ControlSysUI", "DOF", 0));
        Goal_label->setText(QApplication::translate("ControlSysUI", "Goal", 0));
        actmin_label->setText(QApplication::translate("ControlSysUI", "Act_Min", 0));
        actmax_label->setText(QApplication::translate("ControlSysUI", "Act_Max", 0));
        con_KD_label->setText(QApplication::translate("ControlSysUI", "KD", 0));
        con_KI_label->setText(QApplication::translate("ControlSysUI", "KI", 0));
        Controlpara_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Control Parameters</span></p></body></html>", 0));
        con_KP_label->setText(QApplication::translate("ControlSysUI", "KP", 0));
        Fwd_label->setText(QApplication::translate("ControlSysUI", "Fwd", 0));
        fwd_check->setText(QString());
        Depth_label->setText(QApplication::translate("ControlSysUI", "Depth", 0));
        Depth_check->setText(QString());
        yaw_label->setText(QApplication::translate("ControlSysUI", "Yaw", 0));
        yaw_check->setText(QString());
        sm_check->setText(QString());
        sm_label->setText(QApplication::translate("ControlSysUI", "SM", 0));
        enabledButton->setText(QApplication::translate("ControlSysUI", "Enabled", 0));
        disabledButton->setText(QApplication::translate("ControlSysUI", "Disabled", 0));
        Advanced_label->setText(QApplication::translate("ControlSysUI", "<html><head/><body><p><span style=\" font-weight:600;\">Advanced</span></p></body></html>", 0));
        menuFile->setTitle(QApplication::translate("ControlSysUI", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class ControlSysUI: public Ui_ControlSysUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // CONTROLUI_H
