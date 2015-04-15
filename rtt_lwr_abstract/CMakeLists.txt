cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_abstract)

find_package(catkin REQUIRED COMPONENTS rtt_ros kuka_lwr_fri diagnostic_msgs geometry_msgs std_msgs kdl_conversions cmake_modules)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros diagnostic_msgs geometry_msgs std_msgs 
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

set(flags " -DRTT_COMPONENT ")

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  list(APPEND flags  " -DHAVE_RTNET ")
  list(APPEND flags  " -DXENOMAI ")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
set(OROCOS_NO_AUTO_LINKING TRUE)
orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${flags})

orocos_generate_package()

