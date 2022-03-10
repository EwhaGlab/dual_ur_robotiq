cmake_minimum_required(VERSION 3.0.2)
project(pick_place)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


add_executable(pick_place_collision src/pick_place_collision.cpp)
target_link_libraries(pick_place_collision ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS pick_and_place_collision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS pick_and_place_collision_depth_camera DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
