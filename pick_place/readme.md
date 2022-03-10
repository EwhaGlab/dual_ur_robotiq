# Bimanual Pick and Place

Pick and place using predefined pose of table, basket, and block.

![untitled](https://user-images.githubusercontent.com/6389003/157580914-40a7b143-b38b-4920-bb58-b447c7ec6d39.gif)

## Run the lines below in the respective terminals.
```commandline
roscore
roslaunch robotiq_3f_gripper_control dual_gripper_tcp.launch
roslaunch robotiq_3f_gripper_control dual_gripper_controller.launch
roslaunch robotiq_3f_gripper_joint_state_publisher dual_gripper_joint_state_publisher.launch

roslaunch ur_robot_driver robot_bringup.launch   
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

rosrun pick_place pick_place_collision
```