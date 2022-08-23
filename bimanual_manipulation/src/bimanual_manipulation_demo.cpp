#include <ros/ros.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

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
  ros::init(argc, argv, "bimanual_manipulation_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface arms_group_interface("arms");
  // TODO: left and right opposite
  moveit::planning_interface::MoveGroupInterface right_group_interface("left_arm");
  moveit::planning_interface::MoveGroupInterface left_group_interface("right_arm");

  arms_group_interface.setMaxVelocityScalingFactor(1.0);
  arms_group_interface.setMaxAccelerationScalingFactor(1.0);
  arms_group_interface.setPlanningTime(15.0);
  arms_group_interface.setNumPlanningAttempts(20.0);

  moveit::core::RobotModelConstPtr kinematic_model = right_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = arms_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* right_joint_model_group = kinematic_model->getJointModelGroup("left_arm");
  const moveit::core::JointModelGroup* left_joint_model_group = kinematic_model->getJointModelGroup("right_arm");

  const std::vector<std::string>& right_joint_names = right_joint_model_group->getVariableNames();
  const std::vector<std::string>& left_joint_names = left_joint_model_group->getVariableNames();
  std::vector<double> right_joint_values;
  std::vector<double> left_joint_values;

  // init pose
  std::vector<double> init_r = {158*D2R, -119*D2R, -100*D2R, -31*D2R, 50*D2R, -73*D2R};
  std::vector<double> init_l = {-162*D2R, -65*D2R, 99*D2R, -139*D2R, -48*D2R, -25*D2R};
  std::map<std::string, double> variable_values_right = vector_to_map(right_joint_names, init_r);
  std::map<std::string, double> variable_values_left = vector_to_map(left_joint_names, init_l);
  arms_group_interface.setJointValueTarget(variable_values_right);
  arms_group_interface.setJointValueTarget(variable_values_left);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arms_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_INFO("Plan did not successed");
  }
  arms_group_interface.execute(my_plan);
  geometry_msgs::Pose init_right = right_group_interface.getCurrentPose().pose;
  geometry_msgs::Pose init_left = left_group_interface.getCurrentPose().pose;



  // start random posing
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  geometry_msgs::Pose right_pose;
  geometry_msgs::Pose left_pose;

  // w x y z
  Eigen::Quaternionf right_q = Eigen::Quaternionf(init_right.orientation.w, init_right.orientation.x, init_right.orientation.y, init_right.orientation.z);
  Eigen::Quaternionf left_q = Eigen::Quaternionf(init_left.orientation.w, init_left.orientation.x, init_left.orientation.y, init_left.orientation.z);

  for(int i; i < 100; i++){

    float random_x = ( ((float) distr(gen)) * 0.01);
    float random_y = ( ((float) distr(gen)) * 0.01);
    float random_z = ( ((float) distr(gen)) * 0.01);

    right_pose.position.x = init_right.position.x + random_x;
    right_pose.position.y = init_right.position.y + random_y;
    right_pose.position.z = init_right.position.z + random_z;
    left_pose.position.x = init_left.position.x + random_x;
    left_pose.position.y = init_left.position.y + random_y;
    left_pose.position.z = init_left.position.z + random_z;

    float x_rotation = rad_distr(gen) * 0.01;
    float y_rotation = rad_distr(gen) * 0.01;
    float z_rotation = rad_distr(gen) * 0.01;

    right_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                right_q;

    left_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                left_q;

    right_pose.orientation.w = right_q.w();
    right_pose.orientation.x = right_q.x();
    right_pose.orientation.y = right_q.y();
    right_pose.orientation.z = right_q.z();
    left_pose.orientation.w = left_q.w();
    left_pose.orientation.x = left_q.x();
    left_pose.orientation.y = left_q.y();
    left_pose.orientation.z = left_q.z();

    double timeout = 0.1;
    bool right_found_ik = kinematic_state->setFromIK(right_joint_model_group, right_pose, timeout);
    bool left_found_ik = kinematic_state->setFromIK(left_joint_model_group, left_pose, timeout);

    if (right_found_ik && left_found_ik)
    {
      kinematic_state->copyJointGroupPositions(right_joint_model_group, right_joint_values);
      kinematic_state->copyJointGroupPositions(left_joint_model_group, left_joint_values);

      for (std::size_t i = 0; i < right_joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", right_joint_names[i].c_str(), right_joint_values[i]);
        ROS_INFO("Joint %s: %f", left_joint_names[i].c_str(), left_joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    variable_values_right = vector_to_map(right_joint_names, right_joint_values);
    variable_values_left = vector_to_map(left_joint_names, left_joint_values);
    arms_group_interface.setJointValueTarget(variable_values_right);
    arms_group_interface.setJointValueTarget(variable_values_left);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arms_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      ROS_INFO("Plan did not successed");
    }
    arms_group_interface.execute(my_plan);
  }

  ros::shutdown();
}
