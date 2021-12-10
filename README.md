# dual arm ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 18.04 with ROS Melodic.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   

## Gripper 
- Mode: basic, pinch, wide scissor
- Action: close, open   

## Simulation: Run gazebo, moveit, Rviz   
```
roslaunch ur_e_gazebo dual_arm.launch
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch sim:=true
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true
```   

## Real robot execution: Bring up robot, Run moveit and Rviz   
```commandline
roslaunch ur_robot_driver dual_arm_bringup.launch   
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch     
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true   
```

### Gripper
```commandline
rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py  
rosrun robotiq_3f_gripper_control Robotiq3FGripperStatusListener.py
```

### Gripper

Run the lines below in the respective terminals.
```
roscore
rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py 192.168.1.12 //right arm
rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states right_
roslaunch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch
```

## Contact
All bug reports, feedback, comments, contributions or remarks are welcome.
