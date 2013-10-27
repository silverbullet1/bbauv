#/usr/bin/env/sh

#Run this script each time you change the vision.ui file with QT designer

pyuic4 -x vision.ui -o vision.py
