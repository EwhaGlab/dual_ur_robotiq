# Bimanual Manipulation Demo

*Tested on **Ubuntu 18.04** with **ROS Melodic**.*

![dual bimanual_manipulation_demo](https://github.com/EwhaGlab/dual_ur_robotiq/blob/main/etc/img/bimanual_manipulation_demo.gif)

## Random Poses
Demo project for planning and executing random poses for dual arm UR robot simultaneously.

```
roslaunch robot_gazebo dual_ur_robotiq.launch
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch robot_moveit_config moveit_rviz.launch 
rosrun bimanual_manipulation bimanual_manipulation_demo
```

## Pick and Place
Demo project for pick and placing the block.
```
roslaunch robot_gazebo dual_ur_robotiq.launch
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch robot_moveit_config moveit_rviz.launch 
rosrun bimanual_manipulation pick_place_collision
```
