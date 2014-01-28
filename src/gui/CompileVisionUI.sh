#/usr/bin/env/sh

#Run this script each time you change the vision.ui file with QT designer

# pyuic4 -x vision.ui -o src/vision.py
# chmod +x src/vision.py
uic -o src/vision_gui.h vision.ui
uic -o src/controlui.h ControlUI.ui
# chmod +x src/auv_gui.h
