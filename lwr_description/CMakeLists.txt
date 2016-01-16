cmake_minimum_required(VERSION 2.8.3)
project(lwr_description)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY gazebo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/gazebo
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE)