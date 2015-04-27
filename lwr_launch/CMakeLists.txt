cmake_minimum_required(VERSION 2.8.3)
project(lwr_launch)

find_package(catkin REQUIRED COMPONENTS
  lwr_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwr_launch
#  CATKIN_DEPENDS lwr_description rtt_ros rtt_ros_tools rtt_rosparam rtt_sensor_msgs
#  DEPENDS rtt
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

