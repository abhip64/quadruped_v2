cmake_minimum_required(VERSION 2.8.3)
project(quadruped_v2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  geometry_msgs
  joint_state_publisher
  pluginlib
  robot_state_publisher
  roscpp
  rospy
  rviz
  urdf
  xacro
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#    FILES
#    Direction.srv
#   Service2.srv
# )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package( INCLUDE_DIRS src
	LIBRARIES quadruped_v2
	CATKIN_DEPENDS pluginlib roscpp message_runtime
  DEPENDS Boost
)
add_library(lib_walk  src/leg.cpp src/publish.cpp src/gait.cpp src/body.cpp src/path.cpp src/gait_var.cpp src/basic_functions.cpp src/stability.cpp src/body_manip.cpp)

#add_library(quadruped_v2_lib src/leg_rr_controller.cpp)
#target_link_libraries(quadruped_v2_lib ${catkin_LIBRARIES})

add_executable(walk2 src/walk.cpp)

add_dependencies(walk2 quadruped_v2_generate_messages_cpp)

target_link_libraries(walk2 ${catkin_LIBRARIES})
target_link_libraries(walk2 lib_walk)
