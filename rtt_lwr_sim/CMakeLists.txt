cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_sim)

#set(CMAKE_BUILD_TYPE ReleaseWithDebugInfo)

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
  lwr_fri
)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

#set(OROCOS_NO_AUTO_LINKING TRUE)

catkin_package(
   DEPENDS eigen rtt 
   CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs
   )
   
include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${Eigen_INCLUDE_DIRS} 
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)



orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
)


add_dependencies(${PROJECT_NAME} lwr_fri_generate_messages_cpp)
orocos_generate_package(INCLUDE_DIRS include)
