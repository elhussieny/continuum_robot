cmake_minimum_required(VERSION 2.8.3)
project(continuum_robot)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib
 LIBRARIES continuum_robot
)

 include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library 
add_library(${PROJECT_NAME} src/Continuum.cpp include/continuum_robot/Continuum.h)


add_executable(basic_marker src/basic_marker.cpp)
target_link_libraries(basic_marker ${catkin_LIBRARIES})

add_executable(core_node src/core_node.cpp)
target_link_libraries(core_node continuum_robot ${catkin_LIBRARIES})

add_executable(create_cont_robot src/create_cont_robot.cpp)
target_link_libraries(create_cont_robot continuum_robot ${catkin_LIBRARIES})

