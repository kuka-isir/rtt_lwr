cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_example)

find_package(catkin REQUIRED COMPONENTS 
    rtt_ros 
    kuka_lwr_fri 
    diagnostic_msgs 
    geometry_msgs 
    std_msgs 
    kdl_conversions 
    rtt_lwr_abstract 
    cmake_modules
)

catkin_package(
    DEPENDS rtt
    CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${Eigen_INCLUDE_DIRS} 
    ${orocos_kdl_INCLUDE_DIRS}
)

set(OROCOS_NO_AUTO_LINKING TRUE)
orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

if($ENV{OROCOS_TARGET} STREQUAL "xenomai" )
    find_package(Xenomai REQUIRED)
    set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
endif()

target_link_libraries(${PROJECT_NAME} 
    ${orocos_kdl_LIBRARIES} 
    ${kdl_conversions_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${Xenomai_RTDM_LIBRARIES}
)

orocos_generate_package()

