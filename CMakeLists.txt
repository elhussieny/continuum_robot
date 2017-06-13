cmake_minimum_required(VERSION 2.8.3)
project(continuum_robot)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  roslib
   std_msgs
  message_generation
  interactive_markers
 
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime std_msgs
 LIBRARIES continuum_robot
)

 include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library 
add_library(${PROJECT_NAME} src/Continuum.cpp include/continuum_robot/Continuum.h)

add_executable(interface src/interface.cpp)
target_link_libraries(interface ${catkin_LIBRARIES})

add_executable(core_node src/core_node.cpp)
target_link_libraries(core_node continuum_robot ${catkin_LIBRARIES})

add_executable(create_cont_robot src/create_cont_robot.cpp)
target_link_libraries(create_cont_robot continuum_robot ${catkin_LIBRARIES})

