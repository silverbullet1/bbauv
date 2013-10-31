#/usr/bin/env/sh

#Run this script each time you change the vision.ui file with QT designer

# pyuic4 -x vision.ui -o src/vision.py
# chmod +x src/vision.py
uic -o src/auv_gui.h vision.ui
# chmod +x src/auv_gui.h