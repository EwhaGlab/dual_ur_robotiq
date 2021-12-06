# dual arm ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 18.04 with ROS Melodic.

## Build and Compile
1. Clone this repository
```
mkdir robot_ws && cd robot_ws && mkdir src
catkin_init_workspace
cd src
git clone https://github.com/yaesolKim/dual_ur_robotiq.git
```

2. Install the dependencies
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace
```
catkin_make
```


## Usage
1. Run gazebo and spawn the robot model   
```
roslaunch ur_e_gazebo dual_arm.launch
```
2. Run moveit   
```
roslaunch dual_arm_moveit_config robot_moveit_planning_execution.launch sim:=true
```
3. Run Rviz   
```
roslaunch dual_arm_moveit_config moveit_rviz.launch config:=true
```   

The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   


## License

## Contact

All bug reports, feedback, comments, contributions or remarks are welcome.
