# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'vision.ui'
#
# Created: Sun Oct 27 20:01:51 2013
#      by: The PyQt User Interface Compiler (pyuic) 3.18.1
#
# WARNING! All changes made in this file will be lost!


import sys
from qt import *

if __name__ == "__main__":
    a = QApplication(sys.argv)
    QObject.connect(a,SIGNAL("lastWindowClosed()"),a,SLOT("quit()"))
    w = ()
    a.setMainWidget(w)
    w.show()
    a.exec_loop()
