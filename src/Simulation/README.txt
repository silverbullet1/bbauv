** Install uwsim:
- apt-get install ros-hydro-uwsim

** Running uwsim for bumblebee:
- roscore
- cd into this directory
- rosrun uwsim uwsim --configfile ./scenes/sauv_scene.xml
- disableShaders are enabled by default in the scene graph (modify by changing the <disableShaders> tag)

** Main published topics:
- /bumblebee/bumblebee_odom: position of a vehicle on a ROS nav_msgs/Odometry topic
- /bumblebee/camera1: image from front camera
- /bumblebee/camera2: image from bottom camera

** Subscribed topics:
- /dataNavigator: listens on a ROS nav_msgs/Odometry topic and applies the position/velocity to the vehicle

** Test Node (used when the simulation is running):
- Used to control the vehicle velocities using arrow and WASDQE keys
- In the root dir: source ./devel/setup.bash
- rosrun Simulation keyboard_control
- UP: move forward; DOWN: move backward; LEFT & RIGHT: yaw; A & D: roll; W & S: pitch; Q: sink down; E: float up
- view front camera image: rosrun image_view image_view image:=bumblebee/camera1
- view bottom camera image: rosrun image_view image_view image:=bumblebee/camera2
