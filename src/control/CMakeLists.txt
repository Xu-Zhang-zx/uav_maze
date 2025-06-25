cmake_minimum_required(VERSION 3.0.2)
project(control)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(NLOPT_INCLUDE_DIR "/usr/local/include")  
set(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
  gazebo_msgs
  message_generation
)

find_package(Eigen3 REQUIRED) 

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(control
  src/control.cpp
)

target_link_libraries(control
  ${catkin_LIBRARIES}
)

add_executable(timer
  src/timer.cpp
)

target_link_libraries(timer
  ${catkin_LIBRARIES}
)