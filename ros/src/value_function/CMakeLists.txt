cmake_minimum_required(VERSION 2.8.3)
project(value_function)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

#if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/value_function/modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED)
find_package(Matio REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  utils
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    utils
    meta_planner_msgs
    message_runtime
  DEPENDS
    EIGEN3
    MATIO
    BOOST
)

option(ENABLE_DEBUG_MESSAGES "Turn on to enable bound checking and debug messages" ON)
if(ENABLE_DEBUG_MESSAGES)
  add_definitions(-DENABLE_DEBUG_MESSAGES=1)
else()
  add_definitions(-DENABLE_DEBUG_MESSAGES=0)
endif()

add_definitions(-DPRECOMPUTATION_DIR="${CMAKE_SOURCE_DIR}/meta_planner/precomputation/")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${MATIO_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
  ${MATIO_LIBRARY_DIRS}
  ${BOOST_LIBRARY_DIRS}
)

file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${MATIO_LIBRARIES}
  ${BOOST_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${MATIO_LIBRARIES}
  ${BOOST_LIBRARIES}
)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

file(GLOB_RECURSE ${PROJECT_NAME}_execs exec/*.cpp)
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
foreach(executable ${${PROJECT_NAME}_execs})
  get_filename_component(exec_name ${executable} NAME_WE)
  message(STATUS "Including executable   \"${BoldBlue}${exec_name}${ColorReset}\".")
  add_executable(${exec_name} ${executable})
  add_dependencies(${exec_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${exec_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${MATIO_LIBRARIES}
    ${BOOST_LIBRARIES}
  )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
foreach(executable ${${PROJECT_NAME}_execs})
  get_filename_component(exec_name ${executable} NAME_WE)
  message(STATUS "Including executable   \"${BoldBlue}${exec_name}${ColorReset}\".")
  add_executable(${exec_name} ${executable})
  add_dependencies(${exec_name} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${exec_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${MATIO_LIBRARIES}
    ${BOOST_LIBRARIES}
    )
endforeach()
endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(CATKIN_ENABLE_TESTING)
  file(GLOB test_srcs test/*.cpp)
  foreach(test ${test_srcs})
    get_filename_component(test_no_ext ${test} NAME_WE)
    message("Including test   \"${BoldBlue}${test_no_ext}${ColorReset}\".")
  endforeach()

  catkin_add_gtest(test_${PROJECT_NAME} ${test_srcs})
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${MATIO_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BOOST_LIBRARIES}
  )
endif()
