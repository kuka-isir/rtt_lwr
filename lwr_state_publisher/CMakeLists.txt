cmake_minimum_required(VERSION 2.8.3)
project(lwr_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  kuka_lwr_fri
  rtt_ros
  rtt_rosclock
  rtt_roscomm
  sensor_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Just for Hydro compatibility
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_cpp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_lisp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_py)

orocos_generate_package(
DEPENDS rtt_ros rtt_sensor_msgs
)
