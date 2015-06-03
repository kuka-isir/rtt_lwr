cmake_minimum_required(VERSION 2.8.3)
project(lwr_hw_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
  joint_limits_interface
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES lwr_hw_interface
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(lwr_hw_interface
  src/lwr_hw_interface.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(lwr_hw_interface
  ${catkin_LIBRARIES}
)

