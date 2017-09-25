cmake_minimum_required(VERSION 2.8.3)
project(lwr_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cmake_modules
)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

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
)

orocos_install_headers(
    include/${PROJECT_NAME}/ros_helper.h
)

orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)

set(scripts
scripts/deployer-corba
scripts/deployer-corba-debug
scripts/eval_script
scripts/generate_all_urdf_sdf
scripts/kill_depl
scripts/lwr_runner.ops
scripts/ping_ati
scripts/ping_lwr
scripts/process_args
scripts/rtnet
scripts/rtping
)

install(FILES ${scripts}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE GROUP_EXECUTE OWNER_EXECUTE
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN ".svn" EXCLUDE)
