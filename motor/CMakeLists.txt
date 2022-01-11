cmake_minimum_required(VERSION 3.0.2)
project(motor)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

## Add actions
add_action_files(DIRECTORY action FILES Motor.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib
  DEPENDS Boost
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})






## Install scripts and executables
catkin_install_python(PROGRAMS
  src/motor_action_client_test.py
  src/motor_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



