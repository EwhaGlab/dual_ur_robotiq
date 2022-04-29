# Bimanual Manipulation

Solve the alpha puzzle.

## Run the lines below in the respective terminals.
```commandline
roscore
roslaunch robotiq_3f_gripper_control dual_gripper_tcp.launch
roslaunch robotiq_3f_gripper_control dual_gripper_controller.launch
roslaunch robotiq_3f_gripper_joint_state_publisher dual_gripper_joint_state_publisher.launch

roslaunch ur_robot_driver robot_bringup.launch   
roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

rosrun bimanual_manipulation alpha_puzzle
```