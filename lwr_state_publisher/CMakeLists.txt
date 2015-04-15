cmake_minimum_required(VERSION 2.8.3)
project(lwr_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  kuka_lwr_fri
  rtt_ros
  rtt_rosclock
  rtt_roscomm
  sensor_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)
#catkin_package(
#  DEPENDS rtt
#  CATKIN_DEPENDS rtt_ros sensor_msgs rtt_roscomm
#)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#orocos_use_package( ocl-taskbrowser )
#orocos_use_package( ocl-deployment )
#orocos_use_package( ocl-logging )
#use_orocos(rtt-transport-corba)
#orocos_use_package( rtt-typekit )

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package(
DEPENDS rtt_ros rtt_sensor_msgs
)
