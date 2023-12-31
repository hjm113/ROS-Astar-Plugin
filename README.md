# ROS Astar Planner Plugin
This is the astar planner plugin for global planner, which is the open source package of ROS.

Important Note : To execute the plugin for Turtlebot3 burger, it is crucial to change the inflation radius of the costmap(In case you followed the emanual of turtlebot3 from Robotis).
Otherwise, the planner can malfunction becuase it does not locate the start position of the turtlebot.
I recommend to reduce it so that the free space area becomes larger.
