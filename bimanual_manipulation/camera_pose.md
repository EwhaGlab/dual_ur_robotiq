# Bimanual robot camera pose estimation

Calculate the camera to world transformation using dual arm robot.


To use this source, you should install the ar_track_alvar package in advance.

## Run the lines below in the respective terminals.
```commandline
roscore

roslaunch dual_ur_robotiq_description upload_grippers.launch    
roslaunch dual_ur_robotiq_description bringup_robot.launch    

roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

roslaunch bimanual_manipulation cam_ar_track.launch

rosrun bimanual_manipulation camera_pose_estimate
```

```
 
 
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 /world /camera_link
```