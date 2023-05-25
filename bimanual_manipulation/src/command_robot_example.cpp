#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>

#define D2R M_PI/180
#define R2D 180/M_PI

std::map<std::string, double> vector_to_map (const std::vector<std::string> &joint_names, std::vector<double> &joint_values)
{
  std::map<std::string, double> m;
  for (int i = 0; i < joint_names.size(); i++)
    m.insert({joint_names[i], joint_values[i]});
  return m;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_robot");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //-----------------------------------------------
  // Moveit Setup
  //-----------------------------------------------
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARMS = "arms";
  static const std::string PLANNING_GROUP_ARM_R = "right_arm";
  static const std::string PLANNING_GROUP_ARM_L = "left_arm";
  static const std::string PLANNING_GROUP_GRIPPER_R = "right_gripper";
  static const std::string PLANNING_GROUP_GRIPPER_L = "left_gripper";
  static const std::string EE_LINK_R = "right_gripper_tool0";
  static const std::string EE_LINK_L = "left_gripper_tool0";

  // The planning_interface `MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface arms(PLANNING_GROUP_ARMS);
  moveit::planning_interface::MoveGroupInterface rightArm(PLANNING_GROUP_ARM_R);
  moveit::planning_interface::MoveGroupInterface rightGripper(PLANNING_GROUP_GRIPPER_R);
  moveit::planning_interface::MoveGroupInterface leftArm(PLANNING_GROUP_ARM_L);
  moveit::planning_interface::MoveGroupInterface leftGripper(PLANNING_GROUP_GRIPPER_L);

  // set end effector
  rightArm.setEndEffectorLink(EE_LINK_R);
  leftArm.setEndEffectorLink(EE_LINK_L);

  // for compute cartesian path
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;

  // get joint names
  moveit::core::RobotModelConstPtr kinematic_model = rightArm.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = arms.getCurrentState();
  const moveit::core::JointModelGroup* right_joint_model_group = kinematic_model->getJointModelGroup("right_arm");
  const moveit::core::JointModelGroup* left_joint_model_group = kinematic_model->getJointModelGroup("left_arm");
  const std::vector<std::string>& right_joint_names = right_joint_model_group->getVariableNames();
  const std::vector<std::string>& left_joint_names = left_joint_model_group->getVariableNames();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;

  bool success;
  double fraction;

  //-----------------------------------------------------------
  // Method 1-1: Commanding with joint values
  // setJointValue Target arm by arm with joint names
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 1-1: Commanding with joint values");
  ROS_INFO("setJointValue Target arm by arm with joint names");


  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("right_shoulder_pan_joint", -148*D2R);
  rightArm.setJointValueTarget("right_shoulder_lift_joint", -148*D2R);
  rightArm.setJointValueTarget("right_elbow_joint", -53*D2R);
  rightArm.setJointValueTarget("right_wrist_1_joint", -49*D2R);
  rightArm.setJointValueTarget("right_wrist_2_joint", -49*D2R);
  rightArm.setJointValueTarget("right_wrist_3_joint", 107*D2R);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  rightArm.execute(my_plan);

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget("left_shoulder_pan_joint", 154*D2R);
  leftArm.setJointValueTarget("left_shoulder_lift_joint", -29*D2R);
  leftArm.setJointValueTarget("left_elbow_joint", 45*D2R);
  leftArm.setJointValueTarget("left_wrist_1_joint", -121*D2R);
  leftArm.setJointValueTarget("left_wrist_2_joint", 47*D2R);
  leftArm.setJointValueTarget("left_wrist_3_joint", -20*D2R);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  leftArm.execute(my_plan);

  ros::Duration(3).sleep(); // wait for 3 sec

  //-----------------------------------------------------------
  // Method 1-2: Commanding with joint values
  // setJointValue Target arm by arm using saved joint names
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 1-2: Commanding with joint values");
  ROS_INFO("setJointValue Target arm by arm using saved joint names");

  std::vector<double> init_r = {-158*D2R, -148*D2R, -53*D2R, -49*D2R, -49*D2R, 107*D2R};
  std::vector<double> init_l = {164*D2R, -29*D2R, 45*D2R, -121*D2R, 47*D2R, -20*D2R};

  std::map<std::string, double> variable_values_right = vector_to_map(right_joint_names, init_r);
  std::map<std::string, double> variable_values_left = vector_to_map(left_joint_names, init_l);

  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget(variable_values_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  rightArm.execute(my_plan);

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget(variable_values_left);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  leftArm.execute(my_plan);

  ros::Duration(3).sleep(); // wait for 3 sec

  //-----------------------------------------------------------
  // Method 1-3: Commanding with joint values
  // setting specific joint value
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 1-3: Commanding with joint values");
  ROS_INFO("setting specific joint value");

  rightArm.setStartStateToCurrentState();
  std::vector<double> command_joint_value = rightArm.getCurrentJointValues();
  rightArm.setJointValueTarget(right_joint_names[5], command_joint_value[5]-1.57);
  // rightArm.setJointValueTarget("right_wrist_3_joint", command_joint_value[5]-1.57); // same as above
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  rightArm.execute(my_plan);

  ros::Duration(3).sleep(); // wait for 3 sec

  //-----------------------------------------------------------
  // Method 1-4: Commanding with joint values
  // setJointValue Target both arms at once with joint names
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 1-4: Commanding with joint values");
  ROS_INFO("setJointValue Target both arms at once with joint names");


  arms.setStartStateToCurrentState();
  arms.setJointValueTarget("right_shoulder_pan_joint", -168*D2R);
  arms.setJointValueTarget("right_shoulder_lift_joint", -148*D2R);
  arms.setJointValueTarget("right_elbow_joint", -53*D2R);
  arms.setJointValueTarget("right_wrist_1_joint", -49*D2R);
  arms.setJointValueTarget("right_wrist_2_joint", -49*D2R);
  arms.setJointValueTarget("right_wrist_3_joint", 107*D2R);

  arms.setJointValueTarget("left_shoulder_pan_joint", 174*D2R);
  arms.setJointValueTarget("left_shoulder_lift_joint", -29*D2R);
  arms.setJointValueTarget("left_elbow_joint", 45*D2R);
  arms.setJointValueTarget("left_wrist_1_joint", -121*D2R);
  arms.setJointValueTarget("left_wrist_2_joint", 47*D2R);
  arms.setJointValueTarget("left_wrist_3_joint", -20*D2R);
  success = (arms.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  arms.execute(my_plan);

  ros::Duration(3).sleep(); // wait for 3 sec

  //-----------------------------------------------------------
  // Method 1-5: Commanding with joint values
  // setJointValue Target both arms at once using saved joint names
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 2-5: Commanding with joint values");
  ROS_INFO("setJointValue Target arm by arm using saved joint names");

  arms.setStartStateToCurrentState();
  arms.setJointValueTarget(variable_values_right);
  arms.setJointValueTarget(variable_values_left);
  success = (arms.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  arms.execute(my_plan);

  ros::Duration(3).sleep(); // wait for 3 sec


  //-----------------------------------------------------------
  // Method 2-1: Commanding with target pose
  // Motion planning using OMPL planners - robot_moveit_config\ompl_planning.yaml
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 2-1: Commanding with target pose");
  ROS_INFO("Motion planning using OMPL planners");

  // set OMPL planner - refer to robot_moveit_config\ompl_planning.yaml
  rightArm.setPlannerId("RRTConnect");

  geometry_msgs::Pose init_pose_right = rightArm.getCurrentPose(EE_LINK_R).pose;
  geometry_msgs::Pose init_pose_left = leftArm.getCurrentPose(EE_LINK_L).pose;

  geometry_msgs::Pose command_pose_right = init_pose_right;
  command_pose_right.position.x = -0.55;
  command_pose_right.position.y = 0.1;
  command_pose_right.position.z = 1.2;

  command_pose_right.orientation.x = 0.0;
  command_pose_right.orientation.y = 0.0;
  command_pose_right.orientation.z = -0.7071068;
  command_pose_right.orientation.w = 0.7071068;

  // set end effector link
  rightArm.setEndEffectorLink(EE_LINK_R);
  rightArm.setPoseTarget(command_pose_right);

  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  rightArm.move();

  ros::Duration(3).sleep(); // wait for 3 sec


  //-----------------------------------------------------------
  // Method 2-2: Commanding with target pose
  // Compute linear cartesian path using IK solvers - robot_moveit_config\kinematics.yaml
  //-----------------------------------------------------------
  ROS_INFO("----------------------------------------------");
  ROS_INFO("Method 2-2: Commanding with target pose");
  ROS_INFO("Compute linear cartesian path using IK solvers");

  std::vector<geometry_msgs::Pose> linear_path;
  command_pose_right.position.z += 0.1;
  linear_path.push_back(command_pose_right);

  // set end effector link
  rightArm.setEndEffectorLink(EE_LINK_R);
  // compute cartesian path
  fraction = rightArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  if (fraction == 1.0 ) rightArm.execute(my_plan);

  linear_path.clear();

  ros::Duration(3).sleep(); // wait for 3 sec


  ros::shutdown();
  return 0;
}

