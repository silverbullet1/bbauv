Overall Mission Planner

To run: 
-roscore
- rqt (for you to see what topics are being published) => Open Publisher and add the topic /core_node/current_task, then you can see the task being published and type std_msgs/String
- rosrun core core