// 0. initialization
// 1. grasp the marker blocks
// 2. move the block and track the marker (P1)
// 3. move the block again and track the marker (P2)
// 4. calculate the world-camera transformation matrix
// 5. save the transformation matrix as txt file


// ??. locate the camera using the publisher like below
// rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 /world /camera_link


#include <ros/ros.h>
#include <std_msgs/Char.h>

#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <fstream>

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

// TODO: calculate camera-robot transformation matrix
void arMarkersCallback(ar_track_alvar_msgs::AlvarMarkers req)
{
    if (!req.markers.empty()) {
//      tf::Quaternion q(req.markers[0].pose.pose.position.x, req.markers[0].pose.pose.position.y, req.markers[0].pose.pose.position.z);
      tf::Quaternion q(req.markers[1].pose.pose.orientation.x, req.markers[1].pose.pose.orientation.y, req.markers[1].pose.pose.orientation.z, req.markers[1].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
    } // if
}

int main(int argc, char** argv)
{
  std::ofstream file;
  file.open("./dual_ur_robotiq/cam_robot_transformation.txt");

  ros::init(argc, argv, "pick_place");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher gripper_pub_left = n.advertise<std_msgs::Char>("left_gripper/gripper_left", 10);
  ros::Publisher gripper_pub_right = n.advertise<std_msgs::Char>("right_gripper/gripper_right", 10);

  static const std::string PLANNING_GROUP_ARM_RIGHT = "right_arm";
  static const std::string PLANNING_GROUP_ARM_LEFT = "left_arm";

  moveit::planning_interface::MoveGroupInterface rightArm(PLANNING_GROUP_ARM_RIGHT);
  moveit::planning_interface::MoveGroupInterface leftArm(PLANNING_GROUP_ARM_LEFT);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  bool success;

  geometry_msgs::Pose right_goal_pose, left_goal_pose;
  std_msgs::Char gripper_msg_l, gripper_msg_r;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(rightArm.getPlanningFrame().c_str());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

//  ros::Subscriber sub_marker = n.subscribe("/ar_pose_marker", 100, arMarkersCallback);

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(rightArm.getJointModelGroupNames().begin(),
  rightArm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  ROS_INFO_NAMED("tutorial", "\nEnd effector link: %s", rightArm.getEndEffectorLink().c_str());
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  //activate left and right grippers

  gripper_msg_l.data = 'r'; // reset
  gripper_msg_r.data = 'r'; // reset
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  gripper_msg_l.data = 'a'; // activate
  gripper_msg_r.data = 'a'; // activate
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  gripper_msg_l.data = 'p'; // start with pinch mode
  gripper_msg_r.data = 'p'; // start with pinch mode
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  gripper_msg_l.data = 'o'; // open
  gripper_msg_r.data = 'o'; // open
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  // 0-1. test motion : move the right gripper up 1 cm
//  geometry_msgs::PoseStamped current_pose_left = leftArm.getCurrentPose("left_gripper_tool0");
  geometry_msgs::PoseStamped current_pose_right = rightArm.getCurrentPose("right_gripper_tool0");
  std::cout << std::endl <<"right pos: " << current_pose_right.pose.position.x << ", " << current_pose_right.pose.position.y << ", " << current_pose_right.pose.position.z << std::endl;
  std::cout <<"ori: " << current_pose_right.pose.orientation.x << ", " << current_pose_right.pose.orientation.y << ", " << current_pose_right.pose.orientation.z << ", " << current_pose_right.pose.orientation.w << std::endl;

  geometry_msgs::Pose start_pose_left = current_pose_left.pose;
  geometry_msgs::Pose start_pose_right = current_pose_right.pose;

  start_pose_right.position.z += 0.01;
  rightArm.setPoseTarget(start_pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("0-1. Press 'next' to test the right arm move");
  rightArm.move();

  // 0-2. Add Collision object
  ROS_INFO_NAMED("tutorial", "Add objects into the world");
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = rightArm.getPlanningFrame();
  collision_object.id = "table";
  shape_msgs::SolidPrimitive primitive = setPrim(3, 1.7, 1.8, 0.03);
  geometry_msgs::Pose table_pose = setGeomPose(-1.0, 0, 1.20, 0.0, 1.0, 0.0, 0.0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  moveit_msgs::CollisionObject object_to_attach;
  // add left block
  object_to_attach.header.frame_id = leftArm.getPlanningFrame();
  object_to_attach.id = "left_block";
  primitive = setPrim(3, 0.07, 0.07, 0.07);
  geometry_msgs::Pose left_block_pose = setGeomPose(-0.764824, -0.4, 1.25, 0.0, 0.7071068, 0.0, 0.7071068);
  object_to_attach.primitives.push_back(primitive);
  object_to_attach.primitive_poses.push_back(left_block_pose);
  object_to_attach.operation = object_to_attach.ADD;
  collision_objects.push_back(object_to_attach);

  // add right block
  object_to_attach.header.frame_id = rightArm.getPlanningFrame();
  object_to_attach.id = "right_block";
  primitive = setPrim(3, 0.07, 0.07, 0.07);
  geometry_msgs::Pose right_block_pose = setGeomPose(-0.764824, 0.4, 1.25, 0.0, 0.7071068, 0.0, 0.7071068);
  object_to_attach.primitives.push_back(primitive);
  object_to_attach.primitive_poses.push_back(right_block_pose);
  object_to_attach.operation = object_to_attach.ADD;
  collision_objects.push_back(object_to_attach);

  planning_scene_interface.addCollisionObjects(collision_objects);

  visual_tools.publishText(text_pose, "Add objects", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("0-2. Press 'next' in the RvizVisualToolsGui window to once the table appears in RViz");

  right_goal_pose = right_block_pose;
  left_goal_pose = left_block_pose;

  // 0-3. initialize the arm
  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("right_shoulder_pan_joint", -0.26179); //-15
  rightArm.setJointValueTarget("right_shoulder_lift_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("right_elbow_joint", 1.91986); //110
  rightArm.setJointValueTarget("right_wrist_1_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("right_wrist_2_joint", 2.35619); //135
  rightArm.setJointValueTarget("right_wrist_3_joint", -0.523599);  //-30
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing initialization %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("0-3. Press 'next' once the plan is complete. Right arm moving to the initial pose);
  rightArm.execute(my_plan);

  // 1. grasp the right marker block
  // 1-1. Move to the grasping-ready pose
  right_goal_pose.position.z += 0.1;
  rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("1-1. Press 'next' once the plan is complete. Right arm moving to the grasping pose");
  rightArm.move();

  // 1-2. Move down to grasping pose
  right_goal_pose.position.z -= 0.04;
  rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.trigger();
  visual_tools.prompt("1-2. Press 'next' once the plan is complete. Right arm moving down");
  rightArm.execute(my_plan);

  // 1-3. Close the gripper
  visual_tools.trigger();
  visual_tools.prompt("1-3. Press 'next' to close the right gripper");
  gripper_msg_r.data = 'c';
  gripper_pub_right.publish(gripper_msg_r);

  // remove objects from the planning scene
  std::vector<std::string> object_ids;
  object_ids.push_back("left_block");
  object_ids.push_back("right_block");
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 1-4. pick up the marker blocks
  right_goal_pose = rightArm.getCurrentPose("right_gripper_tool0").pose;
  right_goal_pose.position.z += 0.10;
  rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.trigger();
  visual_tools.prompt("1-4. Press 'next' to pick up the right block");
  rightArm.execute(my_plan);

  // 2. move the right marker block (P1)
  // 2-1. rotate the wrist
  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("right_wrist_2_joint", -1.04); //-60
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Rotating the wrist %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("2-1. Press 'next' to rotate the wrist");
  rightArm.move();

  // 2-2. move the right marker block to P1
  right_goal_pose.position.x = -0.64;
  right_goal_pose.position.y = 0.08;
  right_goal_pose.position.z = 1.65;

  right_goal_pose.orientation.x = 0.0;
  right_goal_pose.orientation.y = -0.7071068;
  right_goal_pose.orientation.z = 0.0;
  right_goal_pose.orientation.w = 0.7071068;

  rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // repeat until it succeeds
  while (!success)
  {
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("2-2. Press 'next' once the plan is complete. Move the block to P1");
  rightArm.move();


  //TODO: subscribe marker pose


  // 3. move the right marker block (P2)
  right_goal_pose.position.x = -0.38;
  right_goal_pose.position.y = 0.2;
  right_goal_pose.position.z = 1.86;

  right_goal_pose.orientation.x = 0.5926;
  right_goal_pose.orientation.y = -0.3856;
  right_goal_pose.orientation.z = -0.3856;
  right_goal_pose.orientation.w = 0.5926;

  rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // repeat until it succeeds
  while (!success)
  {
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    rightArm.setApproximateJointValueTarget(right_goal_pose, "right_gripper_tool0");
    success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("3. Press 'next' once the plan is complete. Move the block to P2");
  rightArm.move();

  //TODO: subscribe marker pose


  // FINAL. move to initial pose
  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("right_shoulder_pan_joint", -0.26179); //-15
  rightArm.setJointValueTarget("right_shoulder_lift_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("right_elbow_joint", 1.91986); //110
  rightArm.setJointValueTarget("right_wrist_1_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("right_wrist_2_joint", 2.35619); //135
  rightArm.setJointValueTarget("right_wrist_3_joint", -0.523599);  //-30
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing initialization %s", success ? "" : "FAILED");

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to test the right arm move");
  rightArm.execute(my_plan);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to open the right gripper");
  gripper_msg_r.data = 'o';
  gripper_pub_right.publish(gripper_msg_r);


  //TODO: calculate camera to robot base transformation
  //TODO: save the transformation to txt file

  file << "";
  file.close();

  ros::shutdown();
  return 0;
}


shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z)
{
    shape_msgs::SolidPrimitive pr;
    pr.type = pr.BOX;
    pr.dimensions.resize(d);
    pr.dimensions[pr.BOX_X] = x;
    pr.dimensions[pr.BOX_Y] = y;
    pr.dimensions[pr.BOX_Z] = z;

    return pr;
}

geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow)
{
    geometry_msgs::Pose p;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = ox;
    p.orientation.y = oy;
    p.orientation.z = oz;
    p.orientation.w = ow;

    return p;
}
