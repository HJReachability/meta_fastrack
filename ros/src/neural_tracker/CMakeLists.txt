cmake_minimum_required(VERSION 2.8.3)
project(neural_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  crazyflie_msgs
  crazyflie_utils
  meta_planner_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    visualization_msgs
    geometry_msgs
    std_msgs
    tf2_ros
    crazyflie_msgs
    crazyflie_utils
    meta_planner_msgs
  DEPENDS
)
