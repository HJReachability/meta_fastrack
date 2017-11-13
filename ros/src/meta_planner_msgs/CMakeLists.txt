cmake_minimum_required(VERSION 2.8.3)
project(meta_planner_msgs)

#if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR}/msg ${PROJECT_SOURCE_DIR}/msg/*.msg)
add_message_files(DIRECTORY msg FILES ${msg_files})

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)
