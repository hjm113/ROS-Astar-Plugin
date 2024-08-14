# ROS Astar Plugin
This is the astar planner plugin for the move_base package, which is the open source navigation stack in ROS.

To implement the astar planner, add it in move_base.launch in turtlebot3_navigation package.

```xml
Add <param name="base_global_palnner"value="astar_planner/AstarPlanner" />
```

Astar planner can be checked in Rviz.
