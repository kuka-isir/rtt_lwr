cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_sim)

find_package(catkin REQUIRED COMPONENTS
  kuka_lwr_fri
  rtt_ros
  rtt_rosclock
  rtt_roscomm
  rtt_rosparam
  rtt_lwr_gazebo
  kdl_typekit
  eigen_typekit
  sensor_msgs
  std_msgs
  cmake_modules
  rtt_sensor_msgs
  kdl_parser
)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})


orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)

target_link_libraries(
    ${PROJECT_NAME}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${catkin_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
)

orocos_generate_package(
  DEPENDS rtt_ros rtt_sensor_msgs rtt_std_msgs rtt_geometry rtt_rosparam kdl_parser CartesianImpedance FriJointImpedance
)
