# ROS Astar Plugin
This is the astar planner plugin for the move_base package, which is the open source navigation stack in ROS.


The astar planner has been implemented using turtlebot3, https://emanual.robotis.com

Add the following parameter in move_base.launch in turtlebot3_navigation package to use the astar planner.

```xml
Add <param name="base_global_palnner"value="astar_planner/AstarPlanner" />
```

Astar planner can be checked in Rviz.
