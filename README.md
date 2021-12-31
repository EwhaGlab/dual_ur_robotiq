# dual arm ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 18.04 with ROS Melodic.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   


## Simulation: Run gazebo, moveit, Rviz   
```
roslaunch ur_e_gazebo dual_ur_robotiq.launch
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true
```   
## pick and place simulation
```commandline
rosrun pick_place pick_place_collision
```

## Real robot execution: Bring up grippers and robots, Run moveit and Rviz   
Run the lines below in the respective terminals.
```commandline
roscore
roslaunch robotiq_3f_gripper_control dual_gripper_tcp.launch
roslaunch robotiq_3f_gripper_joint_state_publisher dual_gripper_joint_state_publisher.launch
roslaunch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch

roslaunch ur_robot_driver dual_ur5e_bringup.launch   
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch     
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true   

-------------------------------------------------------------------------
roslaunch ur_robot_driver left_ur5e_bringup.launch   
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch   
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true  
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
