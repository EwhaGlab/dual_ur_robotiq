# dual arm ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 18.04 with ROS Melodic.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   

## Gripper 
- Mode: basic, pinch, wide scissor
- Action: close, open   

## Simulation: Run gazebo, moveit, Rviz
### Single arm simulation
```
roslaunch ur_e_gazebo ur5e.launch
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```   

### Dual arm simulation
```
roslaunch ur_e_gazebo dual_arm.launch
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true
```   


## Real robot control
### Single arm control
```
--
```   

### Dual arm control
```
--
```   


## Contact
All bug reports, feedback, comments, contributions or remarks are welcome.
