cmake_minimum_required(VERSION 2.8.3)
project(lwr_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cmake_modules
)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
link_libraries( ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} )

orocos_use_package(rtt-scripting)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

orocos_service(ros_helper src/ros_helper.cpp)

target_link_libraries(
    ros_helper
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}
)


orocos_install_headers(
        include/${PROJECT_NAME}/ros_helper.h
    )
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN ".svn" EXCLUDE)
