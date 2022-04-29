# Alpha Puzzle

## Run the lines below in the respective terminals.
```commandline
roscore

roslaunch dual_ur_robotiq_description upload_grippers.launch    
roslaunch dual_ur_robotiq_description bringup_robot.launch    

roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

rosrun bimanual_manipulation alpha_puzzle
```


1. calculate the camera to base transformation
2. find the grasp pose? or use pre-defined pose

3. grasp the puzzle, anyway, stale
4. move the arms to locate the puzzle in initial pose
5. calibration??
   1) marker should be attached to the puzzle
   2) calculate the ee to puzzle transforms (left and right independently)
6. generate ee path for follow the alpha puzzle solution trajectory (move only one arm at first.)
7. execute the path