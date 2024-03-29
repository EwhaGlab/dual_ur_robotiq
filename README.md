# Dual arm robot ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 18.04 with ROS Melodic.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   


## Robot simulation
### Run gazebo, moveit, Rviz   
```
roslaunch robot_gazebo dual_ur_robotiq.launch world:=table_box
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch robot_moveit_config moveit_rviz.launch 
```   
### Pick and place simulation
```commandline
rosrun bimanual_manipulation pick_place_collision
```

## Real robot execution: Bring up grippers and robots, Run moveit and Rviz   
Run the lines below in the respective terminals.

```commandline
roscore
roslaunch robotiq_3f_gripper_control dual_gripper_tcp.launch
roslaunch robotiq_3f_gripper_joint_state_publisher dual_gripper_joint_state_publisher.launch
roslaunch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch

roslaunch ur_robot_driver robot_bringup.launch   
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch   
```

## Gripper 
- Mode: basic, pinch, wide scissor
- Action: close, open   

```commandline
rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py  
rosrun robotiq_3f_gripper_control Robotiq3FGripperStatusListener.py
```

## Contact
All bug reports, feedback, comments, contributions or remarks are welcome.
