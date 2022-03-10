#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_collision");
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

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(rightArm.getPlanningFrame().c_str()); //here might be the problem?
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

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
  std_msgs::Char gripper_msg_l, gripper_msg_r;
  gripper_msg_l.data = 'w'; // start with wide mode
  gripper_msg_r.data = 'p'; // start with pinch mode
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  gripper_msg_l.data = 'o'; // open
  gripper_msg_r.data = 'o'; // open
  gripper_pub_left.publish(gripper_msg_l);
  gripper_pub_right.publish(gripper_msg_r);

  //0. test motion
  geometry_msgs::PoseStamped current_pose_left = leftArm.getCurrentPose("left_gripper_tool0");
  std::cout << std::endl <<"left pos: " << current_pose_left.pose.position.x << ", " << current_pose_left.pose.position.y << ", " << current_pose_left.pose.position.z << std::endl;
  std::cout <<"ori: " << current_pose_left.pose.orientation.x << ", " << current_pose_left.pose.orientation.y << ", " << current_pose_left.pose.orientation.z << ", " << current_pose_left.pose.orientation.w << std::endl;

  geometry_msgs::PoseStamped current_pose_right = rightArm.getCurrentPose("right_gripper_tool0");
  std::cout << std::endl <<"right pos: " << current_pose_right.pose.position.x << ", " << current_pose_right.pose.position.y << ", " << current_pose_right.pose.position.z << std::endl;
  std::cout <<"ori: " << current_pose_right.pose.orientation.x << ", " << current_pose_right.pose.orientation.y << ", " << current_pose_right.pose.orientation.z << ", " << current_pose_right.pose.orientation.w << std::endl;

  geometry_msgs::Pose start_pose_right = current_pose_right.pose;
  start_pose_right.position.z += 0.01;
  rightArm.setPoseTarget(start_pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to test the right arm move");
  rightArm.move();

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to plan left arm move");

  geometry_msgs::Pose start_pose_left = current_pose_left.pose;
  start_pose_left.position.z += 0.01;
  leftArm.setPoseTarget(start_pose_left);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to test the left arm move");
  leftArm.move();

  // 1-2. Add Collision object
  ROS_INFO_NAMED("tutorial", "Add objects into the world");
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = rightArm.getPlanningFrame();
  collision_object.id = "table";
  shape_msgs::SolidPrimitive primitive = setPrim(3, 1.7, 4.0, 0.03);
  geometry_msgs::Pose table_pose = setGeomPose(-1.0, 0, 0.95, 0.0, 1.0, 0.0, 0.0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  // add basket
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.header.frame_id = leftArm.getPlanningFrame();
  collision_object.id = "basket";
  primitive = setPrim(3, 0.07, 0.07, 0.07);
  geometry_msgs::Pose basket_pose = setGeomPose(-0.631758, -1.03046, 1.00, 0.0, 0.0, 0.0, 1.0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(basket_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  // add block
  object_to_attach.header.frame_id = rightArm.getPlanningFrame();
  object_to_attach.id = "block";
  primitive = setPrim(3, 0.07, 0.07, 0.07);
  geometry_msgs::Pose block_pose = setGeomPose(-0.764824, 0.734106, 1.00, 0.0, 0.7071068, 0, 0.7071068);
  object_to_attach.primitives.push_back(primitive);
  object_to_attach.primitive_poses.push_back(block_pose);
  object_to_attach.operation = object_to_attach.ADD;
  collision_objects.push_back(object_to_attach);

  planning_scene_interface.addCollisionObjects(collision_objects);

  visual_tools.publishText(text_pose, "Add objects", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("1. Press 'next' in the RvizVisualToolsGui window to once the table appears in RViz");

  // 2. Place the right EE above the wood block
  geometry_msgs::Pose pre_grasp_pose = start_pose_right;
  pre_grasp_pose.position = block_pose.position;
  pre_grasp_pose.position.z += 0.2;

  rightArm.setApproximateJointValueTarget(pre_grasp_pose, "right_gripper_tool0");
  //rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("2. Press 'next' once the plan is complete and then it will move the arm");
  rightArm.move();

  pre_grasp_pose.position.z -= 0.04;
  rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  pre_grasp_pose.position.z -= 0.03;
  rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  pre_grasp_pose.position.z -= 0.02;
  rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  pre_grasp_pose.position.z -= 0.01;
  rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  pre_grasp_pose.position.z -= 0.01;
  rightArm.setPoseTarget(pre_grasp_pose);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  // 3. Close the right gripper
  visual_tools.trigger();
  visual_tools.prompt("3. Press 'next' to close the right gripper");
  gripper_msg_r.data = 'c';
  gripper_pub_right.publish(gripper_msg_r);

  // 4. Place the left EE above the basket
  geometry_msgs::Pose place_pose = start_pose_left;
  place_pose.position = basket_pose.position;
  place_pose.position.z += 0.2;

  leftArm.setApproximateJointValueTarget(place_pose, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("4. Press 'next' once the plan is complete and then it will move the left arm");
  leftArm.execute(my_plan);

  place_pose.position.z -= 0.03;
  leftArm.setApproximateJointValueTarget(place_pose, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.execute(my_plan);

  place_pose.position.z -= 0.02;
  leftArm.setApproximateJointValueTarget(place_pose, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.execute(my_plan);

  place_pose.position.z -= 0.02;
  leftArm.setApproximateJointValueTarget(place_pose, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.execute(my_plan);

  place_pose.position.z -= 0.01;
  leftArm.setApproximateJointValueTarget(place_pose, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.execute(my_plan);


  // 5. Close the left gripper
  visual_tools.trigger();
  visual_tools.prompt("5. Press 'next' to close the left gripper");
  gripper_msg_l.data = 'c';
  gripper_pub_left.publish(gripper_msg_l);

  // 5-1. remove objects from the planning scene
  std::vector<std::string> object_ids;
  object_ids.push_back("block");
  object_ids.push_back("basket");
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 6. pick up both block and basket
  current_pose_right = rightArm.getCurrentPose("right_gripper_tool0");
  current_pose_left = leftArm.getCurrentPose("left_gripper_tool0");

  geometry_msgs::Pose pose_right = current_pose_right.pose;
  geometry_msgs::Pose pose_left = current_pose_left.pose;

  visual_tools.trigger();
  visual_tools.prompt("6. Press 'next' to pick up the obejcts");

  pose_right.position.z += 0.02;
  rightArm.setPoseTarget(pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();
  pose_left.position.z += 0.02;
  leftArm.setPoseTarget(pose_left);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.move();

  pose_right.position.z += 0.02;
  rightArm.setPoseTarget(pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();
  pose_left.position.z += 0.02;
  leftArm.setPoseTarget(pose_left);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.move();

  pose_right.position.z += 0.01;
  rightArm.setPoseTarget(pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();
  pose_left.position.z += 0.01;
  leftArm.setPoseTarget(pose_left);
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  leftArm.move();

  // 7. move the basket
  visual_tools.trigger();
  visual_tools.prompt("7. Press 'next' to plan the pick up motion for left arm");

  pose_left.position.x = -0.784004;
  pose_left.position.y = -0.366719;
  pose_left.position.z = 1.12456;

  pose_left.orientation.x = -0.77;
  pose_left.orientation.y = -0.00;
  pose_left.orientation.z = 0.63;
  pose_left.orientation.w = 0.10;

  leftArm.setApproximateJointValueTarget(pose_left, "left_gripper_tool0");
  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("8. Press 'next' to move left arm to the center of the robot");
  leftArm.move();

  //move the block
  pose_right.position.z += 0.25;
  rightArm.setPoseTarget(pose_right);
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  rightArm.move();

  pose_right.position.x = -0.77;
  pose_right.position.y = 0.00;

  rightArm.setApproximateJointValueTarget(pose_right, "right_gripper_tool0");
  success = (rightArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' once the plan is complete and then it will move the arm");
  rightArm.move();

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to open!");

  gripper_msg_r.data = 'o'; // open
  gripper_pub_right.publish(gripper_msg_r);

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
