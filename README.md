# Dual arm robot ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger adaptive gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)


The project was developed and tested with the following system:

- Ubuntu 18.04    
- ROS Melodic
<!--
UR5e system version 4.2.1
-->

### The features and usage of this driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   

<!--
## Simulation: Run gazebo, moveit, Rviz   
```
roslaunch ur_e_gazebo dual_arm.launch
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true
```   
-->
## Real robot execution

### 1. Start roscore
```commandline
roscore
```
### 2. Bring up the robot
```commandline
roslaunch dual_ur_robotiq_description upload_grippers.launch    
roslaunch dual_ur_robotiq_description bringup_robot.launch    
```
### 3. Run Moveit! and Rviz
```commandline
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   
```

<!--
To control the left/right arm separately, use the command below. 
```commandline
roslaunch ur_robot_driver dual_ur_robotiq_bringup.launch   
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch   
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true  
```
-->

## References   
- [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Robotiq](https://github.com/ros-industrial/robotiq)



## Contact
All bug reports, feedback, comments, contributions or remarks are welcome.
