#!/usr/bin/python
import rospy
from vision import Ui_Vision
from PyQt4 import QtCore, QtGui

def myTopicName1Callback(data):
	rospy.loginfo('myTopicName1Callback is called')

def init_subscribers():    
    rospy.Subscriber("myTopicName1", Image, myTopicName1Callback)

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Vision = QtGui.QMainWindow()
    ui = Ui_Vision()
    ui.setupUi(Vision)
    Vision.show()
    rospy.init_node('GUI_AUV', anonymous=False)
    sys.exit(app.exec_())