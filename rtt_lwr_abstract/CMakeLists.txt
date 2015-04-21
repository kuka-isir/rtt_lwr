cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_abstract)

find_package(catkin REQUIRED COMPONENTS rtt_ros kuka_lwr_fri lwr_fri diagnostic_msgs geometry_msgs std_msgs kdl_conversions cmake_modules)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs 
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
set(OROCOS_NO_AUTO_LINKING TRUE)
orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS eigen)
   
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
  target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
endif()

add_dependencies(${PROJECT_NAME} lwr_fri_gencpp)
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package()

