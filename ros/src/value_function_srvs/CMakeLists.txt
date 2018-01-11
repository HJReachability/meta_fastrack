cmake_minimum_required(VERSION 2.8.3)
project(value_function_srvs)

#if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  meta_planner_msgs
  message_generation
)

file(GLOB srv_files RELATIVE ${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)
add_service_files(DIRECTORY srv FILES ${srv_files})

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  meta_planner_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    meta_planner_msgs
    message_runtime
)
