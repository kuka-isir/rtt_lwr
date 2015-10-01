cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_gazebo)
set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    rtt_ros 
    cmake_modules 
    sensor_msgs 
    message_generation 
    std_msgs
)

catkin_package()

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(Eigen REQUIRED)
#find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  )


orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
endif()

## Default component
orocos_component(lwr_gazebo src/lwr_gazebo_component.cpp)
set_target_properties(lwr_gazebo PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(lwr_gazebo ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GAZEBO_LIBRARIES})

orocos_generate_package()