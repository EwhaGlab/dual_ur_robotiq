# Bimanual Manipulation Demo

![dual bimanual_manipulation_demo](https://github.com/EwhaGlab/dual_ur_robotiq/blob/main/etc/img/dual_arm_demo.gif?raw=true)

Demo project for planning and executing random poses for dual arm UR robot simultaneously. 
Tested with Ubuntu 18.04 and ROS Melodic.

## Demo
Run in each terminal
```
roslaunch robot_gazebo dual_ur_robotiq.launch
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch robot_moveit_config moveit_rviz.launch 
rosrun bimanual_manipulation bimanual_manipulation_demo
```
