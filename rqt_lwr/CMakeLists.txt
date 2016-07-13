cmake_minimum_required(VERSION 2.8.3)
project(rqt_lwr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/rqt_lwr
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/rqt_lwr
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
