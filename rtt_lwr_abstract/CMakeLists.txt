cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_abstract)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rtt_ros
    kuka_lwr_fri
    diagnostic_msgs
    geometry_msgs
    std_msgs
    kdl_conversions
    eigen_conversions
    lwr_fri
    cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)


orocos_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
    ${Xenomai_RTDM_LIBRARIES}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
)


add_dependencies(${PROJECT_NAME} lwr_fri_generate_messages_cpp)
orocos_install_headers(
        include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
    )
orocos_generate_package(INCLUDE_DIRS include
                        DEPENDS rtt_ros_kdl_tools)
