cmake_minimum_required(VERSION 2.8.3)
project(ilqgames)
find_package(catkin REQUIRED)
catkin_package()

include(ExternalProject)

externalproject_add(ilqgames
  GIT_REPOSITORY https://github.com/HJReachability/ilqgames
  GIT_TAG master
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DSHARED_LIBS=ON
  UPDATE_COMMAND ""
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
