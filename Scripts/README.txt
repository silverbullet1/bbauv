INSTRUCTIONS FOR RUNNING SCRIPTS
=====================================
1. ONLY RUN THE SCRIPT IN THE "SCRIPT" FOLDER OR IT WILL NOT WORK
I.E. Run the script in the "Scripts" folder / Open a terminal that 'cd' to the "Scripts" folder

This folder contains scripts that help in setting up the ROS system
1. Run InstallRosHydro.sh will download and set up ROS Hydro in your system
2. Run SetupCatkinWorkspace.sh will make a catkin workspace in this directory
3. Run SetAutoSource.sh will add the appropriate lines to source the workspace in your .bashrc
4. Run InstallAliases.sh will set up aliases for faster typing in your system. To view the aliases, read ros_bash_aliases 
5. Run LocalDNSMap.sh will set up a local DNS mapping for bumblebee's ssh server  (If required)
6. Run InstallQTTools.sh to set up QT5 Designer if you're doing GUI programming. Remember to update your cmake version to 2.8.9 via Ubuntu package updates

For Control Systems/ Power Systems/ Embedded Systems/ Acoustics Guys: 
- Don't need to do Step 4 & 5.
- Continue further by creating your own specific ROS Nodes in the build folder


