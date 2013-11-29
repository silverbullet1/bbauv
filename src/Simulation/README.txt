** Install uwsim:
- apt-get install ros-hydro-uwsim

** Running uwsim here:
- roscore
- cd into this directory
- rosrun uwsim uwsim --configfile ./scenes/sauv_scene.xml
- disableShaders are enabled by default in the scene graph (modify by changing the <disableShaders> tag)

** Main published topics:
- /uwsim/bumblebee_odom: position of a vehicle on a ROS nav_msgs/Odometry topic
- /uwsim/camera1: image from front camera
- /uwsim/camera2: image from bottom camera

** Subscribed topics:
- /dataNavigator: listens on a ROS nav_msgs/Odometry topic and applies the position/velocity to the vehicle

** Test Node:
