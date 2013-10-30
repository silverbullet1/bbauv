/********************************************************************************
** Form generated from reading UI file 'vision.ui'
**
** Created: Wed Oct 30 21:18:41 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef AUV_GUI_H
#define AUV_GUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Vision
{
public slots:
    void openFIle(int a);

public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionQuit;
    QWidget *centralwidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *vboxLayout;
    QFrame *frontcam_2;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *_2;
    QFrame *frontcamfiltered;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *_4;
    QFrame *bottomcam_2;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *_3;
    QFrame *bottomcamfiltered;
    QFrame *line;
    QComboBox *frontfilter;
    QComboBox *bottomfilter;
    QLabel *bottomcam;
    QLabel *frontcam;
    QFrame *line_2;
    QFrame *line_3;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Vision)
    {
        if (Vision->objectName().isEmpty())
            Vision->setObjectName(QString::fromUtf8("Vision"));
        Vision->resize(796, 613);
        actionOpen = new QAction(Vision);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionOpen->setSoftKeyRole(QAction::NoSoftKey);
        actionSave = new QAction(Vision);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionQuit = new QAction(Vision);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        centralwidget = new QWidget(Vision);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 50, 331, 211));
        vboxLayout = new QVBoxLayout(verticalLayoutWidget);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        vboxLayout->setContentsMargins(0, 0, 0, 0);
        frontcam_2 = new QFrame(verticalLayoutWidget);
        frontcam_2->setObjectName(QString::fromUtf8("frontcam_2"));
        frontcam_2->setFrameShape(QFrame::StyledPanel);
        frontcam_2->setFrameShadow(QFrame::Raised);

        vboxLayout->addWidget(frontcam_2);

        verticalLayoutWidget_2 = new QWidget(centralwidget);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(410, 50, 341, 211));
        _2 = new QVBoxLayout(verticalLayoutWidget_2);
        _2->setObjectName(QString::fromUtf8("_2"));
        _2->setContentsMargins(0, 0, 0, 0);
        frontcamfiltered = new QFrame(verticalLayoutWidget_2);
        frontcamfiltered->setObjectName(QString::fromUtf8("frontcamfiltered"));
        frontcamfiltered->setFrameShape(QFrame::StyledPanel);
        frontcamfiltered->setFrameShadow(QFrame::Raised);

        _2->addWidget(frontcamfiltered);

        verticalLayoutWidget_3 = new QWidget(centralwidget);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(20, 330, 331, 201));
        _4 = new QVBoxLayout(verticalLayoutWidget_3);
        _4->setObjectName(QString::fromUtf8("_4"));
        _4->setContentsMargins(0, 0, 0, 0);
        bottomcam_2 = new QFrame(verticalLayoutWidget_3);
        bottomcam_2->setObjectName(QString::fromUtf8("bottomcam_2"));
        bottomcam_2->setFrameShape(QFrame::StyledPanel);
        bottomcam_2->setFrameShadow(QFrame::Raised);

        _4->addWidget(bottomcam_2);

        verticalLayoutWidget_4 = new QWidget(centralwidget);
        verticalLayoutWidget_4->setObjectName(QString::fromUtf8("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(410, 330, 341, 201));
        _3 = new QVBoxLayout(verticalLayoutWidget_4);
        _3->setObjectName(QString::fromUtf8("_3"));
        _3->setContentsMargins(0, 0, 0, 0);
        bottomcamfiltered = new QFrame(verticalLayoutWidget_4);
        bottomcamfiltered->setObjectName(QString::fromUtf8("bottomcamfiltered"));
        bottomcamfiltered->setFrameShape(QFrame::StyledPanel);
        bottomcamfiltered->setFrameShadow(QFrame::Raised);

        _3->addWidget(bottomcamfiltered);

        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(-10, 260, 811, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        frontfilter = new QComboBox(centralwidget);
        frontfilter->setObjectName(QString::fromUtf8("frontfilter"));
        frontfilter->setGeometry(QRect(190, 10, 291, 27));
        bottomfilter = new QComboBox(centralwidget);
        bottomfilter->setObjectName(QString::fromUtf8("bottomfilter"));
        bottomfilter->setGeometry(QRect(200, 280, 291, 27));
        bottomcam = new QLabel(centralwidget);
        bottomcam->setObjectName(QString::fromUtf8("bottomcam"));
        bottomcam->setGeometry(QRect(20, 290, 131, 17));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        bottomcam->setFont(font);
        frontcam = new QLabel(centralwidget);
        frontcam->setObjectName(QString::fromUtf8("frontcam"));
        frontcam->setGeometry(QRect(20, 20, 131, 17));
        frontcam->setFont(font);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(370, 40, 20, 221));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(370, 320, 20, 251));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        Vision->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Vision);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 796, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        Vision->setMenuBar(menubar);
        statusbar = new QStatusBar(Vision);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Vision->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addSeparator();
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);

        retranslateUi(Vision);
        QObject::connect(actionQuit, SIGNAL(triggered()), Vision, SLOT(close()));
        QObject::connect(bottomfilter, SIGNAL(currentIndexChanged(int)), Vision, SLOT(openFIle(int)));

        QMetaObject::connectSlotsByName(Vision);
    } // setupUi

    void retranslateUi(QMainWindow *Vision)
    {
        Vision->setWindowTitle(QApplication::translate("Vision", "Vision", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("Vision", "&Open", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionOpen->setToolTip(QApplication::translate("Vision", "Open a .bag file", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionOpen->setShortcut(QApplication::translate("Vision", "Ctrl+O", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("Vision", "&Save", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionSave->setToolTip(QApplication::translate("Vision", "Save a filtered image", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionSave->setShortcut(QApplication::translate("Vision", "Ctrl+S", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("Vision", "&Quit", 0, QApplication::UnicodeUTF8));
        actionQuit->setShortcut(QApplication::translate("Vision", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        frontfilter->clear();
        frontfilter->insertItems(0, QStringList()
         << QApplication::translate("Vision", "Filter 1", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("Vision", "Filter 2", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("Vision", "Filter 3", 0, QApplication::UnicodeUTF8)
        );
        bottomfilter->clear();
        bottomfilter->insertItems(0, QStringList()
         << QApplication::translate("Vision", "Filter 1", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("Vision", "Filter 2", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("Vision", "Filter 3", 0, QApplication::UnicodeUTF8)
        );
        bottomcam->setText(QApplication::translate("Vision", "Bottom Camera", 0, QApplication::UnicodeUTF8));
        frontcam->setText(QApplication::translate("Vision", "Front Camera", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("Vision", "&File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Vision: public Ui_Vision {};
} // namespace Ui

QT_END_NAMESPACE

#endif // AUV_GUI_H
