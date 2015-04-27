cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_example)

find_package(catkin REQUIRED COMPONENTS 
    rtt_ros 
    kuka_lwr_fri 
    diagnostic_msgs 
    geometry_msgs 
    std_msgs 
    kdl_conversions
    lwr_fri
    rtt_lwr_abstract
    cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

set(OROCOS_NO_AUTO_LINKING TRUE)

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

list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_cpp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_lisp)
list(REMOVE_ITEM catkin_LIBRARIES lwr_fri_generate_messages_py)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    #${USE_OROCOS_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
)

message(WARNING "catkin_INCLUDE_DIRS
${catkin_INCLUDE_DIRS}")
message(WARNING "USE_OROCOS_INCLUDE_DIRS
${USE_OROCOS_INCLUDE_DIRS}")
message(WARNING "catkin_LIBRARIES
${catkin_LIBRARIES}")
message(WARNING "USE_OROCOS_LIBRARIES
${USE_OROCOS_LIBRARIES}")
add_dependencies(${PROJECT_NAME} rtt_lwr_abstract)
orocos_generate_package(INCLUDE_DIRS include)
