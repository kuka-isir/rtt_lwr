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
    std_srvs
)

catkin_package(DEPENDS rtt gazebo eigen)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

# Add c++11 for Gazebo >= 6.0
if(${GAZEBO_VERSION})
    if(${GAZEBO_VERSION} GREATER 6)
        list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
    endif()
endif()

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

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
