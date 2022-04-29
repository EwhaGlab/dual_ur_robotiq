# Bimanual Pick and Place

Pick and place using predefined pose of table, basket, and block.

![untitled](https://user-images.githubusercontent.com/6389003/157580914-40a7b143-b38b-4920-bb58-b447c7ec6d39.gif)

## Run the lines below in the respective terminals.
```commandline
roscore

roslaunch dual_ur_robotiq_description upload_grippers.launch    
roslaunch dual_ur_robotiq_description bringup_robot.launch    

roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

rosrun bimanual_manipulation pick_place
```