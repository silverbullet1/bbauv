Vision GUI 
=======================
- Can run using alias: vision
- Can also run using: rosrun GUI GUI_node

Code related stuffs
=======================
Main UI code is in auv_gui.cpp
Filtering code is inside filters.cpp
    - Add function code in filters.cpp and declaration in filters.h
    - Increment the NUM_FILTERS macro
    - Add a corresponding entry into the front_filters or bottom_filters array in filters.cpp
    - Add an comboBox item in vision.ui

Controls GUI 
=======================
- run using: rosrun GUI Control_node
- can modify the "live" parameter in command line as: rosrun GUI Control_node _live:=true
If live is false, parameters initialised to 0; otherwise, it will subscribe to the relevant topics
Default mode is live:=false


